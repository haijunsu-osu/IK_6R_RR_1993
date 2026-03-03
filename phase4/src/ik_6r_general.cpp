#include "ik_6r_general.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "pq_constants.hpp"

namespace rr1993 {

namespace {

using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec14 = Eigen::Matrix<double, 14, 1>;

using Mat6x9 = Eigen::Matrix<double, 6, 9>;
using Mat6x14 = Eigen::Matrix<double, 6, 14>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using Mat14x8 = Eigen::Matrix<double, 14, 8>;
using Mat14x9 = Eigen::Matrix<double, 14, 9>;
using Mat17x4 = Eigen::Matrix<double, 17, 4>;
using Mat17x14 = Eigen::Matrix<double, 17, 14>;
using Mat17x17 = Eigen::Matrix<double, 17, 17>;

constexpr double kAngleTol = 1e-6;
constexpr double kResidualTol = 1e-4;
constexpr double kPi = 3.141592653589793238462643383279502884;

template <typename Derived>
double max_abs(const Eigen::MatrixBase<Derived>& m) {
    return m.cwiseAbs().maxCoeff();
}

double wrap_angle(double theta) {
    double wrapped = std::fmod(theta + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

std::pair<double, double> normalize_sc(double s, double c) {
    const double n = std::hypot(s, c);
    if (n < 1e-14) {
        return {0.0, 1.0};
    }
    return {s / n, c / n};
}

Mat4 aiv(double theta) {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Mat4 out = Mat4::Identity();
    out(0, 0) = c;
    out(0, 1) = -s;
    out(1, 0) = s;
    out(1, 1) = c;
    return out;
}

Mat4 ais(double a, double alpha, double d) {
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    Mat4 out = Mat4::Identity();
    out(0, 3) = a;
    out(1, 1) = ca;
    out(1, 2) = -sa;
    out(2, 1) = sa;
    out(2, 2) = ca;
    out(2, 3) = d;
    return out;
}

Mat4 dh_matrix(double a, double alpha, double d, double theta) {
    return aiv(theta) * ais(a, alpha, d);
}

Mat4 se3_inverse(const Mat4& T) {
    const Eigen::Matrix3d r = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);
    Mat4 out = Mat4::Identity();
    out.block<3, 3>(0, 0) = r.transpose();
    out.block<3, 1>(0, 3) = -(r.transpose() * p);
    return out;
}

const Mat17x4& pq_symbolic_sample_angles() {
    static const Mat17x4 m = []() {
        Mat17x4 out;
        for (int i = 0; i < 17; ++i) {
            for (int j = 0; j < 4; ++j) {
                out(i, j) = kPQSymbolicSampleAngles[static_cast<size_t>(i * 4 + j)];
            }
        }
        return out;
    }();
    return m;
}

const Mat17x17& pq_feature_matrix_inv() {
    static const Mat17x17 m = []() {
        Mat17x17 out;
        for (int i = 0; i < 17; ++i) {
            for (int j = 0; j < 17; ++j) {
                out(i, j) = kPQFeatureMatrixInv[static_cast<size_t>(i * 17 + j)];
            }
        }
        return out;
    }();
    return m;
}

struct PLVectors {
    Vec3 p_left;
    Vec3 p_right;
    Vec3 l_left;
    Vec3 l_right;
};

PLVectors build_pl_vectors(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    const Vec6& q) {
    std::array<Mat4, 6> A;
    for (int i = 0; i < 6; ++i) {
        A[i] = dh_matrix(dh_params(i, 0), dh_params(i, 1), dh_params(i, 2), q(i));
    }
    const Mat4 a2v = aiv(q(1));
    const Mat4 a2s = ais(dh_params(1, 0), dh_params(1, 1), dh_params(1, 2));
    const Mat4 left = a2s * A[2] * A[3] * A[4];
    const Mat4 right = a2v.inverse() * A[0].inverse() * target_pose * A[5].inverse();
    PLVectors out;
    out.p_left = left.block<3, 1>(0, 3);
    out.p_right = right.block<3, 1>(0, 3);
    out.l_left = left.block<3, 1>(0, 2);
    out.l_right = right.block<3, 1>(0, 2);
    return out;
}

Vec14 build_14_equations(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    const Vec6& q) {
    const PLVectors pl = build_pl_vectors(dh_params, target_pose, q);
    const Vec3 eq_p = pl.p_left - pl.p_right;
    const Vec3 eq_l = pl.l_left - pl.l_right;
    const double eq_pp = pl.p_left.dot(pl.p_left) - pl.p_right.dot(pl.p_right);
    const double eq_pl = pl.p_left.dot(pl.l_left) - pl.p_right.dot(pl.l_right);
    const Vec3 eq_px = pl.p_left.cross(pl.l_left) - pl.p_right.cross(pl.l_right);
    const Vec3 combo_l = pl.p_left.dot(pl.p_left) * pl.l_left -
                         2.0 * pl.p_left.dot(pl.l_left) * pl.p_left;
    const Vec3 combo_r = pl.p_right.dot(pl.p_right) * pl.l_right -
                         2.0 * pl.p_right.dot(pl.l_right) * pl.p_right;
    const Vec3 eq_combo = combo_l - combo_r;

    Vec14 out;
    out.segment<3>(0) = eq_p;
    out.segment<3>(3) = eq_l;
    out(6) = eq_pp;
    out(7) = eq_pl;
    out.segment<3>(8) = eq_px;
    out.segment<3>(11) = eq_combo;
    return out;
}

Vec14 equation_values_at_x3(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    double x3,
    double theta1,
    double theta2,
    double theta4,
    double theta5) {
    Vec6 q;
    q << theta1, theta2, 2.0 * std::atan(x3), theta4, theta5, 0.0;
    return build_14_equations(dh_params, target_pose, q);
}

Vec9 monomial_45(double theta4, double theta5) {
    const double s4 = std::sin(theta4);
    const double c4 = std::cos(theta4);
    const double s5 = std::sin(theta5);
    const double c5 = std::cos(theta5);
    Vec9 v;
    v << s4 * s5, s4 * c5, c4 * s5, c4 * c5, s4, c4, s5, c5, 1.0;
    return v;
}

Vec8 monomial_12(double theta1, double theta2) {
    const double s1 = std::sin(theta1);
    const double c1 = std::cos(theta1);
    const double s2 = std::sin(theta2);
    const double c2 = std::cos(theta2);
    Vec8 v;
    v << s1 * s2, s1 * c2, c1 * s2, c1 * c2, s1, c1, s2, c2;
    return v;
}

Mat9 build_halfangle_map_45() {
    Mat9 t = Mat9::Zero();
    t(0, 3) = 4.0;
    t(1, 2) = -2.0;
    t(1, 6) = 2.0;
    t(2, 1) = -2.0;
    t(2, 7) = 2.0;
    t(3, 0) = 1.0;
    t(3, 4) = -1.0;
    t(3, 5) = -1.0;
    t(3, 8) = 1.0;
    t(4, 2) = 2.0;
    t(4, 6) = 2.0;
    t(5, 0) = -1.0;
    t(5, 4) = -1.0;
    t(5, 5) = 1.0;
    t(5, 8) = 1.0;
    t(6, 1) = 2.0;
    t(6, 7) = 2.0;
    t(7, 0) = -1.0;
    t(7, 4) = 1.0;
    t(7, 5) = -1.0;
    t(7, 8) = 1.0;
    t(8, 0) = 1.0;
    t(8, 4) = 1.0;
    t(8, 5) = 1.0;
    t(8, 8) = 1.0;
    return t;
}

Mat17x14 build_y_matrix_at_x3(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    double x3) {
    Mat17x14 values;
    const Mat17x4& sa = pq_symbolic_sample_angles();
    for (int i = 0; i < 17; ++i) {
        const Vec14 e = equation_values_at_x3(
            dh_params, target_pose, x3, sa(i, 0), sa(i, 1), sa(i, 2), sa(i, 3));
        values.row(i) = e.transpose();
    }
    return values;
}

struct PQBuild {
    Mat14x9 P;
    Mat14x8 Q;
};

PQBuild build_pq_at_x3(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    double x3) {
    const Mat17x14 values = build_y_matrix_at_x3(dh_params, target_pose, x3);
    const Mat17x14 coeff = pq_feature_matrix_inv() * values;
    PQBuild out;
    out.P = coeff.topRows<9>().transpose();
    out.Q = coeff.bottomRows<8>().transpose();
    return out;
}

int matrix_rank(const Eigen::MatrixXd& M, double tol) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) {
            ++r;
        }
    }
    return r;
}

std::vector<int> select_independent_rows(const Eigen::MatrixXd& M, int k) {
    std::vector<int> idx;
    std::vector<int> remaining(M.rows());
    for (int i = 0; i < M.rows(); ++i) {
        remaining[i] = i;
    }

    while (static_cast<int>(idx.size()) < k && !remaining.empty()) {
        int best_row = -1;
        double best_score = std::numeric_limits<double>::infinity();
        for (int r : remaining) {
            std::vector<int> cand = idx;
            cand.push_back(r);
            Eigen::MatrixXd block(cand.size(), M.cols());
            for (int i = 0; i < static_cast<int>(cand.size()); ++i) {
                block.row(i) = M.row(cand[i]);
            }
            if (matrix_rank(block, 1e-10) < static_cast<int>(cand.size())) {
                continue;
            }
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(block, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const auto& s = svd.singularValues();
            if (s.size() == 0 || std::abs(s(s.size() - 1)) <= 1e-14) {
                continue;
            }
            const double score = s(0) / s(s.size() - 1);
            if (score < best_score) {
                best_score = score;
                best_row = r;
            }
        }
        if (best_row < 0) {
            break;
        }
        idx.push_back(best_row);
        remaining.erase(std::remove(remaining.begin(), remaining.end(), best_row), remaining.end());
    }
    return idx;
}

bool eliminate_theta12(const Mat14x9& P, const Mat14x8& Q, Mat6x9& E45) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeFullU | Eigen::ComputeThinV);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > 1e-10) {
            ++rank;
        }
    }
    if (rank < 8) {
        return false;
    }
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd left_null = U.rightCols(U.cols() - rank).transpose();
    if (left_null.rows() < 6) {
        return false;
    }
    E45 = left_null.topRows(6) * P;
    return true;
}

Mat12 build_dialytic_12x12(const Mat6x9& E9) {
    Mat12 M = Mat12::Zero();
    for (int i = 0; i < 6; ++i) {
        const Eigen::Matrix<double, 1, 9> c = E9.row(i);
        Eigen::Matrix<double, 1, 12> r = Eigen::Matrix<double, 1, 12>::Zero();
        r(3) = c(0);
        r(4) = c(1);
        r(6) = c(2);
        r(7) = c(3);
        r(5) = c(4);
        r(9) = c(5);
        r(8) = c(6);
        r(10) = c(7);
        r(11) = c(8);
        M.row(i) = r;

        r.setZero();
        r(0) = c(0);
        r(1) = c(1);
        r(3) = c(2);
        r(4) = c(3);
        r(2) = c(4);
        r(6) = c(5);
        r(5) = c(6);
        r(7) = c(7);
        r(8) = c(8);
        M.row(i + 6) = r;
    }
    return M;
}

Vec12 null_vector(const Mat12& M) {
    Eigen::JacobiSVD<Mat12> svd(M, Eigen::ComputeFullV);
    Vec12 z = svd.matrixV().col(11);
    const double n = z.norm();
    if (n > 0.0) {
        z /= n;
    }
    return z;
}

std::vector<std::pair<double, double>> recover_x4x5_candidates(const Vec12& z) {
    std::vector<std::pair<double, double>> out;
    auto add = [&](double x4, double x5) {
        if (!std::isfinite(x4) || !std::isfinite(x5)) {
            return;
        }
        if (std::abs(x4) > 1e8 || std::abs(x5) > 1e8) {
            return;
        }
        for (const auto& uv : out) {
            if (std::abs(x4 - uv.first) <= 1e-8 && std::abs(x5 - uv.second) <= 1e-8) {
                return;
            }
        }
        out.push_back({x4, x5});
    };
    if (std::abs(z(11)) > 1e-10) {
        add(z(8) / z(11), z(10) / z(11));
    }
    if (std::abs(z(8)) > 1e-10) {
        add(z(5) / z(8), z(7) / z(8));
    }
    if (std::abs(z(10)) > 1e-10) {
        add(z(7) / z(10), z(9) / z(10));
    }
    return out;
}

Vec12 monomial12_from_x4x5(double x4, double x5) {
    Vec12 v;
    v << std::pow(x4, 3) * std::pow(x5, 2), std::pow(x4, 3) * x5, std::pow(x4, 3),
        std::pow(x4, 2) * std::pow(x5, 2), std::pow(x4, 2) * x5, std::pow(x4, 2),
        x4 * std::pow(x5, 2), x4 * x5, x4, std::pow(x5, 2), x5, 1.0;
    return v;
}

bool validate_x4x5_pair(double x4, double x5, const Mat12& M, const Mat6x9& E9) {
    const Vec12 z12 = monomial12_from_x4x5(x4, x5);
    const double eq14_err = max_abs(M * z12);
    if (eq14_err > 1e-5) {
        return false;
    }
    Vec9 z9;
    z9 << x4 * x4 * x5 * x5, x4 * x4 * x5, x4 * x5 * x5, x4 * x5, x4 * x4, x5 * x5, x4, x5, 1.0;
    const double e9_err = max_abs(E9 * z9);
    return e9_err <= 1e-3;
}

std::vector<std::pair<double, double>> solve_x4x5_from_eq14_eigenvectors(
    const Mat12& M,
    const Mat6x9& E9,
    int max_vectors = 8) {
    std::vector<std::pair<double, double>> candidates;
    auto append_unique = [&](double x4, double x5) {
        if (!std::isfinite(x4) || !std::isfinite(x5)) {
            return;
        }
        for (const auto& uv : candidates) {
            if (std::abs(x4 - uv.first) <= 1e-7 && std::abs(x5 - uv.second) <= 1e-7) {
                return;
            }
        }
        candidates.push_back({x4, x5});
    };

    Eigen::EigenSolver<Mat12> es(M, true);
    if (es.info() == Eigen::Success) {
        std::vector<int> order(12);
        for (int i = 0; i < 12; ++i) {
            order[i] = i;
        }
        auto evals = es.eigenvalues();
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return std::abs(evals(a)) < std::abs(evals(b));
        });

        const int lim = std::min(max_vectors, 12);
        for (int kk = 0; kk < lim; ++kk) {
            const int idx = order[kk];
            const Eigen::VectorXcd vec = es.eigenvectors().col(idx);
            double max_im = 0.0;
            for (int i = 0; i < vec.size(); ++i) {
                max_im = std::max(max_im, std::abs(vec(i).imag()));
            }
            if (max_im > 1e-8) {
                continue;
            }
            Vec12 z;
            for (int i = 0; i < 12; ++i) {
                z(i) = vec(i).real();
            }
            const double mz = z.cwiseAbs().maxCoeff();
            if (mz <= 1e-14) {
                continue;
            }
            z /= mz;
            for (const auto& uv : recover_x4x5_candidates(z)) {
                if (validate_x4x5_pair(uv.first, uv.second, M, E9)) {
                    append_unique(uv.first, uv.second);
                }
            }
        }
    }

    if (candidates.empty()) {
        const Vec12 z = null_vector(M);
        for (const auto& uv : recover_x4x5_candidates(z)) {
            if (validate_x4x5_pair(uv.first, uv.second, M, E9)) {
                append_unique(uv.first, uv.second);
            }
        }
    }

    return candidates;
}

std::pair<double, double> recover_theta12(
    const Mat14x9& P,
    const Mat14x8& Q,
    double theta4,
    double theta5) {
    const Vec9 m45 = monomial_45(theta4, theta5);
    const Vec14 rhs = P * m45;
    const Vec8 n = Q.colPivHouseholderQr().solve(rhs);
    const auto [s1, c1] = normalize_sc(n(4), n(5));
    const auto [s2, c2] = normalize_sc(n(6), n(7));
    return {wrap_angle(std::atan2(s1, c1)), wrap_angle(std::atan2(s2, c2))};
}

double recover_theta6(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    double q1,
    double q2,
    double q3,
    double q4,
    double q5) {
    Mat4 T1to5 = Mat4::Identity();
    const std::array<double, 5> q = {q1, q2, q3, q4, q5};
    for (int i = 0; i < 5; ++i) {
        T1to5 = T1to5 * dh_matrix(dh_params(i, 0), dh_params(i, 1), dh_params(i, 2), q[i]);
    }
    const Mat4 A6_target = se3_inverse(T1to5) * target_pose;
    return wrap_angle(std::atan2(A6_target(1, 0), A6_target(0, 0)));
}

double pose_residual(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    const std::array<double, 6>& q) {
    Vec6 vq;
    for (int i = 0; i < 6; ++i) {
        vq(i) = q[static_cast<size_t>(i)];
    }
    const Mat4 fk = forward_kinematics(dh_params, vq);
    return (fk - target_pose).norm();
}

std::vector<IKSolution> dedupe_solutions(const std::vector<IKSolution>& sols, double tol = kAngleTol) {
    std::vector<IKSolution> out;
    for (const auto& sol : sols) {
        bool unique = true;
        for (const auto& existing : out) {
            double n2 = 0.0;
            for (int i = 0; i < 6; ++i) {
                const double d = wrap_angle(sol.q[static_cast<size_t>(i)] - existing.q[static_cast<size_t>(i)]);
                n2 += d * d;
            }
            if (std::sqrt(n2) <= tol) {
                unique = false;
                break;
            }
        }
        if (unique) {
            out.push_back(sol);
        }
    }
    std::sort(out.begin(), out.end(), [](const IKSolution& a, const IKSolution& b) {
        return a.residual < b.residual;
    });
    return out;
}

Eigen::MatrixXd build_left_elimination_matrix(const Mat14x8& Q, const std::vector<int>& pivot_rows) {
    if (pivot_rows.size() != 8) {
        return Eigen::MatrixXd();
    }
    std::vector<int> rem;
    for (int r = 0; r < 14; ++r) {
        if (std::find(pivot_rows.begin(), pivot_rows.end(), r) == pivot_rows.end()) {
            rem.push_back(r);
        }
    }
    if (rem.size() != 6) {
        return Eigen::MatrixXd();
    }

    Eigen::Matrix<double, 8, 8> Qp;
    Eigen::Matrix<double, 6, 8> Qr;
    for (int i = 0; i < 8; ++i) {
        Qp.row(i) = Q.row(pivot_rows[static_cast<size_t>(i)]);
    }
    for (int i = 0; i < 6; ++i) {
        Qr.row(i) = Q.row(rem[static_cast<size_t>(i)]);
    }
    if (matrix_rank(Qp, 1e-10) < 8) {
        return Eigen::MatrixXd();
    }

    Eigen::Matrix<double, 6, 8> X = Qp.transpose().colPivHouseholderQr().solve(Qr.transpose()).transpose();
    Eigen::Matrix<double, 6, 14> Nperm = Eigen::Matrix<double, 6, 14>::Zero();
    Nperm.block<6, 8>(0, 0) = -X;
    Nperm.block<6, 6>(0, 8) = Eigen::Matrix<double, 6, 6>::Identity();

    const std::vector<int> order = [&]() {
        std::vector<int> o = pivot_rows;
        o.insert(o.end(), rem.begin(), rem.end());
        return o;
    }();
    Eigen::Matrix<double, 6, 14> N = Eigen::Matrix<double, 6, 14>::Zero();
    for (int j = 0; j < 14; ++j) {
        N.col(order[static_cast<size_t>(j)]) = Nperm.col(j);
    }
    return N;
}

struct SinCosModel {
    Mat14x9 P_s = Mat14x9::Zero();
    Mat14x9 P_c = Mat14x9::Zero();
    Mat14x9 P_1 = Mat14x9::Zero();
    Mat14x8 Q_const = Mat14x8::Zero();
    double p_fit_max_err = 0.0;
    double q_spread = 0.0;
};

SinCosModel build_pq_sincos_model(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose) {
    SinCosModel model;
    const Eigen::Vector3d t3 = (Eigen::Vector3d() << -2.1, -0.2, 1.3).finished();
    Eigen::Matrix3d basis;
    for (int i = 0; i < 3; ++i) {
        basis(i, 0) = std::sin(t3(i));
        basis(i, 1) = std::cos(t3(i));
        basis(i, 2) = 1.0;
    }
    const Eigen::Matrix3d basis_inv = basis.inverse();

    std::array<Mat14x9, 3> P_samples;
    std::array<Mat14x8, 3> Q_samples;
    for (int i = 0; i < 3; ++i) {
        const double x3 = std::tan(0.5 * t3(i));
        const PQBuild pq = build_pq_at_x3(dh_params, target_pose, x3);
        P_samples[static_cast<size_t>(i)] = pq.P;
        Q_samples[static_cast<size_t>(i)] = pq.Q;
    }

    for (int r = 0; r < 14; ++r) {
        for (int c = 0; c < 9; ++c) {
            Eigen::Vector3d y;
            y << P_samples[0](r, c), P_samples[1](r, c), P_samples[2](r, c);
            const Eigen::Vector3d cc = basis_inv * y;
            model.P_s(r, c) = cc(0);
            model.P_c(r, c) = cc(1);
            model.P_1(r, c) = cc(2);
        }
    }
    model.Q_const = (Q_samples[0] + Q_samples[1] + Q_samples[2]) / 3.0;

    for (int i = 0; i < 3; ++i) {
        const Mat14x9 P_rec = model.P_s * std::sin(t3(i)) + model.P_c * std::cos(t3(i)) + model.P_1;
        model.p_fit_max_err = std::max(model.p_fit_max_err, max_abs(P_rec - P_samples[static_cast<size_t>(i)]));
    }
    model.q_spread = 0.0;
    for (int i = 0; i < 3; ++i) {
        model.q_spread = std::max(model.q_spread, max_abs(Q_samples[static_cast<size_t>(i)] - model.Q_const));
    }
    return model;
}

struct MatrixPolyModel {
    std::array<Mat12, 3> coeff{};  // [C, B, A]
    Mat6x14 N = Mat6x14::Zero();
    double matrix_eval_time = 0.0;
    double fit_time = 0.0;
    double max_entry_err = 0.0;
};

MatrixPolyModel build_matrix_poly_model(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    const std::vector<double>& x_samples) {
    MatrixPolyModel out;

    const auto t0 = std::chrono::high_resolution_clock::now();
    const SinCosModel model = build_pq_sincos_model(dh_params, target_pose);
    out.matrix_eval_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();

    const std::vector<int> piv = select_independent_rows(model.Q_const, 8);
    if (piv.size() != 8) {
        throw std::runtime_error("Failed to pick rank-8 pivot rows from Q.");
    }
    const Eigen::MatrixXd Ndyn = build_left_elimination_matrix(model.Q_const, piv);
    if (Ndyn.rows() != 6 || Ndyn.cols() != 14) {
        throw std::runtime_error("Failed to construct fixed elimination matrix N.");
    }
    out.N = Ndyn;

    const auto t1 = std::chrono::high_resolution_clock::now();
    const Mat9 map45 = build_halfangle_map_45();
    const Mat14x9 P2 = model.P_1 - model.P_c;
    const Mat14x9 P1 = 2.0 * model.P_s;
    const Mat14x9 P0 = model.P_1 + model.P_c;

    const Mat6x9 E9_2 = (out.N * P2) * map45;
    const Mat6x9 E9_1 = (out.N * P1) * map45;
    const Mat6x9 E9_0 = (out.N * P0) * map45;

    const Mat12 M2 = build_dialytic_12x12(E9_2);
    const Mat12 M1 = build_dialytic_12x12(E9_1);
    const Mat12 M0 = build_dialytic_12x12(E9_0);
    out.coeff[0] = M0;
    out.coeff[1] = M1;
    out.coeff[2] = M2;
    out.fit_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t1).count();

    out.max_entry_err = 0.0;
    for (double x : x_samples) {
        const PQBuild pq = build_pq_at_x3(dh_params, target_pose, x);
        const Mat6x9 E9x = (out.N * pq.P) * map45;
        const Mat12 Mx = build_dialytic_12x12(E9x);
        const Mat12 lhs = (1.0 + x * x) * Mx;
        const Mat12 rhs = M0 + M1 * x + M2 * (x * x);
        out.max_entry_err = std::max(out.max_entry_err, max_abs(lhs - rhs));
    }

    return out;
}

std::vector<std::complex<double>> solve_matrix_poly_roots(const std::array<Mat12, 3>& coeff) {
    // Sigma(x) = A x^2 + B x + C, coeff=[C,B,A].
    const Mat12& C = coeff[0];
    const Mat12& B = coeff[1];
    const Mat12& A = coeff[2];

    Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(24, 24);
    Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(24, 24);
    M1.block(0, 12, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    M1.block(12, 0, 12, 12) = -C;
    M1.block(12, 12, 12, 12) = -B;

    M2.block(0, 0, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    M2.block(12, 12, 12, 12) = A;

    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(M1, M2, true);
    std::vector<std::complex<double>> roots;
    if (ges.info() != Eigen::Success) {
        return roots;
    }
    const Eigen::VectorXcd evals = ges.eigenvalues();
    roots.reserve(static_cast<size_t>(evals.size()));
    for (int i = 0; i < evals.size(); ++i) {
        roots.push_back(evals(i));
    }
    return roots;
}

double characteristic_det_at_x3(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Mat4& target_pose,
    double x3) {
    const PQBuild pq = build_pq_at_x3(dh_params, target_pose, x3);
    Mat6x9 E45;
    if (!eliminate_theta12(pq.P, pq.Q, E45)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Mat6x9 E9 = E45 * build_halfangle_map_45();
    const Mat12 M = build_dialytic_12x12(E9);
    return M.determinant();
}

double refine_root_newton(
    const std::function<double(double)>& detf,
    double x0,
    int max_iter = 20) {
    double x = x0;
    for (int it = 0; it < max_iter; ++it) {
        const double fx = detf(x);
        if (!std::isfinite(fx)) {
            break;
        }
        const double h = 1e-4 * (1.0 + std::abs(x));
        const double fp = (detf(x + h) - detf(x - h)) / (2.0 * h);
        if (!std::isfinite(fp) || std::abs(fp) <= 1e-12) {
            break;
        }
        const double xn = x - fx / fp;
        if (!std::isfinite(xn)) {
            break;
        }
        if (std::abs(xn - x) <= 1e-10 * (1.0 + std::abs(x))) {
            x = xn;
            break;
        }
        x = xn;
    }
    return x;
}

std::vector<double> dedupe_scalar_roots(const std::vector<double>& vals, double tol) {
    std::vector<double> s = vals;
    std::sort(s.begin(), s.end());
    std::vector<double> out;
    for (double v : s) {
        if (!std::isfinite(v)) {
            continue;
        }
        if (out.empty() || std::abs(v - out.back()) > tol) {
            out.push_back(v);
        }
    }
    return out;
}

}  // namespace

double normalize_angle(double theta) {
    return wrap_angle(theta);
}

Eigen::Matrix4d forward_kinematics(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Eigen::Matrix<double, 6, 1>& q) {
    Mat4 T = Mat4::Identity();
    for (int i = 0; i < 6; ++i) {
        T = T * dh_matrix(dh_params(i, 0), dh_params(i, 1), dh_params(i, 2), q(i));
    }
    return T;
}

std::vector<IKSolution> solve_ik_6r_general(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Eigen::Matrix4d& target_pose,
    IKDiagnostics* diagnostics,
    double residual_tolerance,
    double matrix_pencil_root_bound_factor) {
    const std::vector<double> x_samples = []() {
        std::vector<double> x(41);
        for (int i = 0; i < 41; ++i) {
            x[static_cast<size_t>(i)] = -6.0 + 12.0 * static_cast<double>(i) / 40.0;
        }
        return x;
    }();

    MatrixPolyModel poly_model;
    try {
        poly_model = build_matrix_poly_model(dh_params, target_pose, x_samples);
    } catch (...) {
        if (diagnostics != nullptr) {
            *diagnostics = IKDiagnostics{};
        }
        return {};
    }

    const auto t_solve0 = std::chrono::high_resolution_clock::now();
    const std::vector<std::complex<double>> roots = solve_matrix_poly_roots(poly_model.coeff);
    int poly_num_roots = static_cast<int>(roots.size());
    int poly_num_real_roots = 0;
    std::vector<double> raw_real;
    const double x_max = std::abs(*std::max_element(
        x_samples.begin(), x_samples.end(),
        [](double a, double b) { return std::abs(a) < std::abs(b); }));
    const double x_bound = std::max(1.0, x_max) * matrix_pencil_root_bound_factor;
    for (const auto& r : roots) {
        if (std::abs(r.imag()) <= 1e-7 && std::isfinite(r.real())) {
            ++poly_num_real_roots;
            if (std::abs(r.real()) <= x_bound) {
                raw_real.push_back(r.real());
            }
        }
    }
    raw_real = dedupe_scalar_roots(raw_real, 1e-4);

    const auto detf = [&](double x) { return characteristic_det_at_x3(dh_params, target_pose, x); };
    std::vector<double> refined;
    refined.reserve(raw_real.size());
    for (double x : raw_real) {
        const double xr = refine_root_newton(detf, x);
        if (std::isfinite(xr) && std::abs(xr) <= x_bound) {
            refined.push_back(xr);
        }
    }
    refined = dedupe_scalar_roots(refined, 1e-5);
    std::vector<double> real_x3;
    if (!refined.empty()) {
        std::vector<std::pair<double, double>> scored;
        scored.reserve(refined.size());
        for (double x : refined) {
            scored.push_back({std::abs(detf(x)), x});
        }
        std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        std::vector<double> near_zero;
        for (const auto& p : scored) {
            if (p.first <= 1e-6) {
                near_zero.push_back(p.second);
            }
        }
        if (!near_zero.empty()) {
            real_x3 = near_zero;
        } else {
            for (const auto& p : scored) {
                real_x3.push_back(p.second);
            }
        }
        real_x3 = dedupe_scalar_roots(real_x3, 1e-5);
    }
    const double stage_poly_solve_time =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t_solve0).count();

    std::vector<IKSolution> candidates;
    const Mat9 map45 = build_halfangle_map_45();
    const auto t_back0 = std::chrono::high_resolution_clock::now();
    for (double x3 : real_x3) {
        const double theta3 = wrap_angle(2.0 * std::atan(x3));
        const PQBuild pq = build_pq_at_x3(dh_params, target_pose, x3);
        Mat6x9 E45;
        if (!eliminate_theta12(pq.P, pq.Q, E45)) {
            continue;
        }
        const Mat6x9 E9 = E45 * map45;
        const Mat12 M = build_dialytic_12x12(E9);
        auto x45 = solve_x4x5_from_eq14_eigenvectors(M, E9);
        if (x45.empty()) {
            for (const auto& uv : recover_x4x5_candidates(null_vector(M))) {
                if (validate_x4x5_pair(uv.first, uv.second, M, E9)) {
                    x45.push_back(uv);
                }
            }
        }

        for (const auto& uv : x45) {
            const double theta4 = wrap_angle(2.0 * std::atan(uv.first));
            const double theta5 = wrap_angle(2.0 * std::atan(uv.second));
            const auto [theta1, theta2] = recover_theta12(pq.P, pq.Q, theta4, theta5);
            const double theta6 = recover_theta6(dh_params, target_pose, theta1, theta2, theta3, theta4, theta5);
            IKSolution sol;
            sol.q = {
                wrap_angle(theta1),
                wrap_angle(theta2),
                wrap_angle(theta3),
                wrap_angle(theta4),
                wrap_angle(theta5),
                wrap_angle(theta6),
            };
            sol.residual = pose_residual(dh_params, target_pose, sol.q);
            if (sol.residual <= residual_tolerance) {
                candidates.push_back(sol);
            }
        }
    }
    const double stage_back_sub_time =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t_back0).count();

    std::vector<IKSolution> solutions = dedupe_solutions(candidates, kAngleTol);
    if (diagnostics != nullptr) {
        diagnostics->num_polynomial_roots = poly_num_roots;
        diagnostics->num_polynomial_real_roots = poly_num_real_roots;
        diagnostics->num_candidate_x3_roots = static_cast<int>(real_x3.size());
        diagnostics->num_ik_solutions = static_cast<int>(solutions.size());
        diagnostics->num_real_ik_solutions = static_cast<int>(solutions.size());
        diagnostics->time_matrix_construction_sec = poly_model.matrix_eval_time;
        diagnostics->time_poly_derivation_sec = poly_model.fit_time;
        diagnostics->time_poly_solve_sec = stage_poly_solve_time;
        diagnostics->time_back_substitution_sec = stage_back_sub_time;
        diagnostics->time_profiled_total_sec = poly_model.matrix_eval_time + poly_model.fit_time +
                                               stage_poly_solve_time + stage_back_sub_time;
    }
    return solutions;
}

}  // namespace rr1993

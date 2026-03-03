#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include "ik_6r_general.hpp"

namespace {

double orientation_error_rad(const Eigen::Matrix3d& rt, const Eigen::Matrix3d& rp) {
    const Eigen::Matrix3d rd = rt.transpose() * rp;
    const double tr = rd.trace();
    const double c = std::clamp(0.5 * (tr - 1.0), -1.0, 1.0);
    return std::acos(c);
}

}  // namespace

int main() {
    constexpr double kPi = 3.141592653589793238462643383279502884;
    Eigen::Matrix<double, 6, 3> dh;
    dh << 0.3, kPi / 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, kPi / 2.0, 0.2, 1.5, 1.0, 0.0, 0.0, kPi / 2.0, 0.0, 0.0,
        1.0, 0.0;

    Eigen::Matrix<double, 6, 1> q_true;
    q_true << 0.0, 107.458 * kPi / 180.0, 112.460 * kPi / 180.0, -7.662 * kPi / 180.0, 0.0, 0.0;

    Eigen::Matrix4d target_published;
    target_published << -0.4655, -0.4822, 0.7421, -0.5377, -0.6987, 0.7150, 0.0263, -1.1561, -0.5433, -0.5063,
        -0.6697, 0.1895, 0.0, 0.0, 0.0, 1.0;

    const Eigen::Matrix4d target_fk = rr1993::forward_kinematics(dh, q_true);
    const double published_diff = (target_fk - target_published).cwiseAbs().maxCoeff();
    if (published_diff >= 5e-4) {
        std::cerr << "FAIL: published target mismatch too large: " << published_diff << "\n";
        return 1;
    }

    rr1993::IKDiagnostics diag;
    const std::vector<rr1993::IKSolution> sols = rr1993::solve_ik_6r_general(dh, target_fk, &diag, 1e-4, 4.0);
    if (diag.num_polynomial_real_roots != 16 || diag.num_candidate_x3_roots != 16) {
        std::cerr << "FAIL: expected 16 real roots/candidates, got roots="
                  << diag.num_polynomial_real_roots
                  << " candidates=" << diag.num_candidate_x3_roots << "\n";
        return 1;
    }
    if (static_cast<int>(sols.size()) < 15) {
        std::cerr << "FAIL: expected at least 15 IK branches, got " << sols.size() << "\n";
        return 1;
    }

    const double pos_th = 1e-6;
    const double ori_th = 1e-6;
    const double tf_th = 2e-6;
    int accepted = 0;
    for (const auto& s : sols) {
        Eigen::Matrix<double, 6, 1> q;
        for (int i = 0; i < 6; ++i) {
            q(i) = s.q[static_cast<size_t>(i)];
        }
        const Eigen::Matrix4d pred = rr1993::forward_kinematics(dh, q);
        const double pos_err = (pred.block<3, 1>(0, 3) - target_fk.block<3, 1>(0, 3)).norm();
        const double ori_err = orientation_error_rad(target_fk.block<3, 3>(0, 0), pred.block<3, 3>(0, 0));
        const double tf_err = (pred - target_fk).norm();
        if (pos_err <= pos_th && ori_err <= ori_th && tf_err <= tf_th) {
            ++accepted;
        }
    }
    if (accepted < 15) {
        std::cerr << "FAIL: accepted FK-IK-FK branches below 15: " << accepted << "\n";
        return 1;
    }

    std::cout << "test_manseur_doty_16 passed. solutions=" << sols.size()
              << ", accepted=" << accepted
              << ", real_roots=" << diag.num_polynomial_real_roots << "\n";
    return 0;
}

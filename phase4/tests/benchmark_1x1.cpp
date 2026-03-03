#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "ik_6r_general.hpp"

namespace {

double orientation_error_rad(const Eigen::Matrix3d& r_target, const Eigen::Matrix3d& r_pred) {
    const Eigen::Matrix3d r_delta = r_target.transpose() * r_pred;
    const double trace_val = r_delta.trace();
    const double cos_angle = std::clamp(0.5 * (trace_val - 1.0), -1.0, 1.0);
    return std::acos(cos_angle);
}

}  // namespace

int main() {
    Eigen::Matrix<double, 6, 3> dh;
    dh << 0.32, 0.70, 0.18, 0.25, -0.90, 0.21, 0.29, 0.80, 0.14, 0.22, -1.10, 0.19, 0.18, 0.60, 0.11, 0.15,
        -0.70, 0.17;

    Eigen::Matrix<double, 6, 1> q_true;
    q_true << 0.60, -1.00, 0.90, -0.80, 1.20, -0.40;
    const Eigen::Matrix4d target = rr1993::forward_kinematics(dh, q_true);

    rr1993::IKDiagnostics diag;
    const auto t0 = std::chrono::high_resolution_clock::now();
    const auto sols = rr1993::solve_ik_6r_general(dh, target, &diag);
    const double wall =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();

    double worst_pos = 0.0;
    double worst_ori = 0.0;
    for (const auto& s : sols) {
        Eigen::Matrix<double, 6, 1> q;
        for (int i = 0; i < 6; ++i) {
            q(i) = s.q[static_cast<size_t>(i)];
        }
        const Eigen::Matrix4d fk = rr1993::forward_kinematics(dh, q);
        const double pos = (target.block<3, 1>(0, 3) - fk.block<3, 1>(0, 3)).norm();
        const double ori = orientation_error_rad(target.block<3, 3>(0, 0), fk.block<3, 3>(0, 0));
        worst_pos = std::max(worst_pos, pos);
        worst_ori = std::max(worst_ori, ori);
    }

    std::cout << "=== C++ 1x1 IK benchmark ===\n";
    std::cout << "wall_time_sec: " << wall << "\n";
    std::cout << "num_solutions: " << sols.size() << "\n";
    std::cout << "num_polynomial_roots / real / candidate_x3: " << diag.num_polynomial_roots << " / "
              << diag.num_polynomial_real_roots << " / " << diag.num_candidate_x3_roots << "\n";
    std::cout << "stage_matrix_construction_sec: " << diag.time_matrix_construction_sec << "\n";
    std::cout << "stage_poly_derivation_sec: " << diag.time_poly_derivation_sec << "\n";
    std::cout << "stage_poly_solve_sec: " << diag.time_poly_solve_sec << "\n";
    std::cout << "stage_back_substitution_sec: " << diag.time_back_substitution_sec << "\n";
    std::cout << "profiled_total_sec: " << diag.time_profiled_total_sec << "\n";
    std::cout << "worst_position_residual: " << worst_pos << "\n";
    std::cout << "worst_orientation_residual_rad: " << worst_ori << "\n";
    return 0;
}


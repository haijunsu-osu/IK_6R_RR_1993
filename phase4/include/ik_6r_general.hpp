#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace rr1993 {

struct IKSolution {
    std::array<double, 6> q{};
    double residual = 0.0;
};

struct IKDiagnostics {
    int num_polynomial_roots = 0;
    int num_polynomial_real_roots = 0;
    int num_candidate_x3_roots = 0;
    int num_ik_solutions = 0;
    int num_real_ik_solutions = 0;

    double time_matrix_construction_sec = 0.0;
    double time_poly_derivation_sec = 0.0;
    double time_poly_solve_sec = 0.0;
    double time_back_substitution_sec = 0.0;
    double time_profiled_total_sec = 0.0;
};

double normalize_angle(double theta);

Eigen::Matrix4d forward_kinematics(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Eigen::Matrix<double, 6, 1>& q);

std::vector<IKSolution> solve_ik_6r_general(
    const Eigen::Matrix<double, 6, 3>& dh_params,
    const Eigen::Matrix4d& target_pose,
    IKDiagnostics* diagnostics = nullptr,
    double residual_tolerance = 1e-4,
    double matrix_pencil_root_bound_factor = 4.0);

}  // namespace rr1993


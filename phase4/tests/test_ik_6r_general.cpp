#include <cmath>
#include <iostream>
#include <vector>

#include "ik_6r_general.hpp"

namespace {

double angle_distance(const std::array<double, 6>& a, const std::array<double, 6>& b) {
    double n2 = 0.0;
    for (int i = 0; i < 6; ++i) {
        const double d = rr1993::normalize_angle(a[static_cast<size_t>(i)] - b[static_cast<size_t>(i)]);
        n2 += d * d;
    }
    return std::sqrt(n2);
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
    const std::vector<rr1993::IKSolution> sols = rr1993::solve_ik_6r_general(dh, target, &diag);

    if (sols.empty()) {
        std::cerr << "FAIL: no IK solutions found\n";
        return 1;
    }
    std::array<double, 6> q_true_arr{};
    for (int i = 0; i < 6; ++i) {
        q_true_arr[static_cast<size_t>(i)] = q_true(i);
    }

    double best_dist = 1e9;
    double best_res = 1e9;
    for (const auto& s : sols) {
        best_dist = std::min(best_dist, angle_distance(s.q, q_true_arr));
        best_res = std::min(best_res, s.residual);
    }
    if (best_res >= 1e-3) {
        std::cerr << "FAIL: best residual too large: " << best_res << "\n";
        return 1;
    }
    if (best_dist >= 0.3) {
        std::cerr << "FAIL: best angular distance too large: " << best_dist << "\n";
        return 1;
    }

    std::cout << "test_ik_6r_general passed. solutions=" << sols.size() << "\n";
    return 0;
}

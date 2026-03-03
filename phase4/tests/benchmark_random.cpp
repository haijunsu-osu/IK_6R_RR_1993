#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "ik_6r_general.hpp"

namespace {

struct Config {
    int num_dh_sets = 100;
    int samples_per_dh = 10;
    std::uint64_t seed = 20260214ULL;
    std::string output_json = "tests/benchmarks/benchmark_cpp_random_results.json";
    std::string output_md = "tests/benchmarks/benchmark_cpp_random_results.md";
};

struct CaseStat {
    double ik_time_sec = 0.0;
    int num_solutions = 0;
    int num_real_solutions = 0;
    int num_poly_roots = 0;
    int num_poly_real_roots = 0;
    int accepted_solutions = 0;
    double best_position_residual = std::numeric_limits<double>::infinity();
    double best_orientation_residual_rad = std::numeric_limits<double>::infinity();
    double best_transform_residual = std::numeric_limits<double>::infinity();
    double t_matrix = 0.0;
    double t_poly_derivation = 0.0;
    double t_poly_solve = 0.0;
    double t_back_sub = 0.0;
};

double orientation_error_rad(const Eigen::Matrix3d& r_target, const Eigen::Matrix3d& r_pred) {
    const Eigen::Matrix3d r_delta = r_target.transpose() * r_pred;
    const double tr = r_delta.trace();
    const double c = std::clamp(0.5 * (tr - 1.0), -1.0, 1.0);
    return std::acos(c);
}

double quantile(std::vector<double> v, double q) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(v.begin(), v.end());
    const double pos = q * static_cast<double>(v.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const auto hi = static_cast<std::size_t>(std::ceil(pos));
    if (lo == hi) {
        return v[lo];
    }
    const double t = pos - static_cast<double>(lo);
    return (1.0 - t) * v[lo] + t * v[hi];
}

double mean(const std::vector<double>& v) {
    if (v.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double s = std::accumulate(v.begin(), v.end(), 0.0);
    return s / static_cast<double>(v.size());
}

double median(std::vector<double> v) {
    return quantile(std::move(v), 0.5);
}

void ensure_parent_dir(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }
}

Config parse_args(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto read_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + name);
            }
            ++i;
            return std::string(argv[i]);
        };
        if (arg == "--num-dh-sets") {
            cfg.num_dh_sets = std::stoi(read_value(arg));
        } else if (arg == "--samples-per-dh") {
            cfg.samples_per_dh = std::stoi(read_value(arg));
        } else if (arg == "--seed") {
            cfg.seed = static_cast<std::uint64_t>(std::stoull(read_value(arg)));
        } else if (arg == "--output-json") {
            cfg.output_json = read_value(arg);
        } else if (arg == "--output-md") {
            cfg.output_md = read_value(arg);
        } else {
            throw std::runtime_error("Unknown arg: " + arg);
        }
    }
    return cfg;
}

}  // namespace

int main(int argc, char** argv) {
    Config cfg;
    try {
        cfg = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        return 2;
    }

    std::mt19937_64 rng(cfg.seed);
    std::uniform_real_distribution<double> dist_a(0.05, 0.60);
    std::uniform_real_distribution<double> dist_alpha(-3.14159265358979323846, 3.14159265358979323846);
    std::uniform_real_distribution<double> dist_d(0.02, 0.50);
    std::uniform_real_distribution<double> dist_q(-3.14159265358979323846, 3.14159265358979323846);

    const int total_cases = cfg.num_dh_sets * cfg.samples_per_dh;
    std::vector<CaseStat> cases;
    cases.reserve(static_cast<std::size_t>(total_cases));

    std::cout << "Running C++ benchmark: " << cfg.num_dh_sets << " DH sets x " << cfg.samples_per_dh
              << " samples = " << total_cases << " IK queries\n";
    std::cout << "IK timing includes only solve_ik_6r_general; pose generation/data setup are excluded.\n";

    int counter = 0;
    for (int dh_idx = 0; dh_idx < cfg.num_dh_sets; ++dh_idx) {
        Eigen::Matrix<double, 6, 3> dh;
        for (int i = 0; i < 6; ++i) {
            dh(i, 0) = dist_a(rng);
            dh(i, 1) = dist_alpha(rng);
            dh(i, 2) = dist_d(rng);
        }
        for (int sample_idx = 0; sample_idx < cfg.samples_per_dh; ++sample_idx) {
            Eigen::Matrix<double, 6, 1> q_true;
            for (int i = 0; i < 6; ++i) {
                q_true(i) = dist_q(rng);
            }
            const Eigen::Matrix4d target = rr1993::forward_kinematics(dh, q_true);  // excluded from timing

            rr1993::IKDiagnostics diag;
            const auto t0 = std::chrono::high_resolution_clock::now();
            const auto sols = rr1993::solve_ik_6r_general(dh, target, &diag);
            const double ik_time =
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();

            CaseStat c;
            c.ik_time_sec = ik_time;
            c.num_solutions = static_cast<int>(sols.size());
            c.num_real_solutions = diag.num_real_ik_solutions;
            c.num_poly_roots = diag.num_polynomial_roots;
            c.num_poly_real_roots = diag.num_polynomial_real_roots;
            constexpr double kPosThreshold = 1e-4;
            constexpr double kOriThreshold = 1e-4;
            constexpr double kTfThreshold = 3e-4;
            for (const auto& s : sols) {
                Eigen::Matrix<double, 6, 1> q_sol;
                for (int i = 0; i < 6; ++i) {
                    q_sol(i) = s.q[static_cast<size_t>(i)];
                }
                const Eigen::Matrix4d pred = rr1993::forward_kinematics(dh, q_sol);
                const double pos_err = (pred.block<3, 1>(0, 3) - target.block<3, 1>(0, 3)).norm();
                const double ori_err = orientation_error_rad(target.block<3, 3>(0, 0), pred.block<3, 3>(0, 0));
                const double tf_err = (pred - target).norm();
                c.best_position_residual = std::min(c.best_position_residual, pos_err);
                c.best_orientation_residual_rad = std::min(c.best_orientation_residual_rad, ori_err);
                c.best_transform_residual = std::min(c.best_transform_residual, tf_err);
                if (pos_err <= kPosThreshold && ori_err <= kOriThreshold && tf_err <= kTfThreshold) {
                    ++c.accepted_solutions;
                }
            }
            c.t_matrix = diag.time_matrix_construction_sec;
            c.t_poly_derivation = diag.time_poly_derivation_sec;
            c.t_poly_solve = diag.time_poly_solve_sec;
            c.t_back_sub = diag.time_back_substitution_sec;
            cases.push_back(c);

            ++counter;
            if (counter % 25 == 0 || counter == total_cases) {
                std::cout << "[progress] " << counter << "/" << total_cases << " ("
                          << std::fixed << std::setprecision(1)
                          << (100.0 * static_cast<double>(counter) / static_cast<double>(total_cases)) << "%)\n";
            }
        }
    }

    std::vector<double> t_ik, t_matrix, t_poly_deriv, t_poly_solve, t_back_sub;
    std::vector<double> n_sol, n_real_sol, n_poly_real;
    t_ik.reserve(cases.size());
    t_matrix.reserve(cases.size());
    t_poly_deriv.reserve(cases.size());
    t_poly_solve.reserve(cases.size());
    t_back_sub.reserve(cases.size());
    n_sol.reserve(cases.size());
    n_real_sol.reserve(cases.size());
    n_poly_real.reserve(cases.size());

    int solved_cases = 0;
    int accepted_cases = 0;
    for (const auto& c : cases) {
        t_ik.push_back(c.ik_time_sec);
        t_matrix.push_back(c.t_matrix);
        t_poly_deriv.push_back(c.t_poly_derivation);
        t_poly_solve.push_back(c.t_poly_solve);
        t_back_sub.push_back(c.t_back_sub);
        n_sol.push_back(static_cast<double>(c.num_solutions));
        n_real_sol.push_back(static_cast<double>(c.num_real_solutions));
        n_poly_real.push_back(static_cast<double>(c.num_poly_real_roots));
        if (c.num_solutions > 0) {
            ++solved_cases;
        }
        if (c.accepted_solutions > 0) {
            ++accepted_cases;
        }
    }

    const double ik_total = std::accumulate(t_ik.begin(), t_ik.end(), 0.0);

    ensure_parent_dir(cfg.output_json);
    ensure_parent_dir(cfg.output_md);

    {
        std::ofstream f(cfg.output_json, std::ios::out | std::ios::trunc);
        f << std::fixed << std::setprecision(10);
        f << "{\n";
        f << "  \"summary\": {\n";
        f << "    \"seed\": " << cfg.seed << ",\n";
        f << "    \"num_dh_sets\": " << cfg.num_dh_sets << ",\n";
        f << "    \"samples_per_dh\": " << cfg.samples_per_dh << ",\n";
        f << "    \"total_cases\": " << total_cases << ",\n";
        f << "    \"root_solver\": \"matrix_polynomial_pencil\",\n";
        f << "    \"cases_with_solution\": " << solved_cases << ",\n";
        f << "    \"cases_with_accepted_solution\": " << accepted_cases << ",\n";
        f << "    \"acceptance_rate\": "
          << (total_cases > 0 ? static_cast<double>(accepted_cases) / static_cast<double>(total_cases) : 0.0)
          << ",\n";
        f << "    \"ik_time_total_sec\": " << ik_total << ",\n";
        f << "    \"ik_time_mean_sec\": " << mean(t_ik) << ",\n";
        f << "    \"ik_time_median_sec\": " << median(t_ik) << ",\n";
        f << "    \"ik_time_p90_sec\": " << quantile(t_ik, 0.90) << ",\n";
        f << "    \"ik_time_p95_sec\": " << quantile(t_ik, 0.95) << ",\n";
        f << "    \"ik_time_max_sec\": " << *std::max_element(t_ik.begin(), t_ik.end()) << ",\n";
        f << "    \"solutions_per_case_mean\": " << mean(n_sol) << ",\n";
        f << "    \"real_solutions_per_case_mean\": " << mean(n_real_sol) << ",\n";
        f << "    \"poly_real_roots_per_case_mean\": " << mean(n_poly_real) << ",\n";
        f << "    \"stage_matrix_construction_mean_sec\": " << mean(t_matrix) << ",\n";
        f << "    \"stage_poly_derivation_mean_sec\": " << mean(t_poly_deriv) << ",\n";
        f << "    \"stage_poly_solve_mean_sec\": " << mean(t_poly_solve) << ",\n";
        f << "    \"stage_back_substitution_mean_sec\": " << mean(t_back_sub) << "\n";
        f << "  }\n";
        f << "}\n";
    }

    {
        std::ofstream f(cfg.output_md, std::ios::out | std::ios::trunc);
        f << std::fixed << std::setprecision(6);
        f << "# C++ IK 6R Random Benchmark Summary\n\n";
        f << "- Root solver: `matrix_polynomial_pencil`\n";
        f << "- DH sets: `" << cfg.num_dh_sets << "`\n";
        f << "- Samples per DH set: `" << cfg.samples_per_dh << "`\n";
        f << "- Total IK queries: `" << total_cases << "`\n";
        f << "- Random seed: `" << cfg.seed << "`\n\n";
        f << "## Aggregate Metrics\n\n";
        f << "| Metric | Value |\n";
        f << "|---|---:|\n";
        f << "| Cases with >=1 IK solution | " << solved_cases << "/" << total_cases << " ("
          << (100.0 * static_cast<double>(solved_cases) / static_cast<double>(total_cases)) << "%) |\n";
        f << "| Cases with >=1 accepted FK-IK-FK solution | " << accepted_cases << "/" << total_cases << " ("
          << (100.0 * static_cast<double>(accepted_cases) / static_cast<double>(total_cases)) << "%) |\n";
        f << "| Mean IK time (s) | " << mean(t_ik) << " |\n";
        f << "| Median IK time (s) | " << median(t_ik) << " |\n";
        f << "| P90 IK time (s) | " << quantile(t_ik, 0.90) << " |\n";
        f << "| P95 IK time (s) | " << quantile(t_ik, 0.95) << " |\n";
        f << "| Max IK time (s) | " << *std::max_element(t_ik.begin(), t_ik.end()) << " |\n";
        f << "| Total IK time (s) | " << ik_total << " |\n";
        f << "| Mean solutions obtained per case | " << mean(n_sol) << " |\n";
        f << "| Mean real solutions obtained per case | " << mean(n_real_sol) << " |\n";
        f << "| Mean polynomial real roots per case | " << mean(n_poly_real) << " |\n";
        f << "| Mean stage: matrix construction (s) | " << mean(t_matrix) << " |\n";
        f << "| Mean stage: polynomial derivation (s) | " << mean(t_poly_deriv) << " |\n";
        f << "| Mean stage: polynomial solve (s) | " << mean(t_poly_solve) << " |\n";
        f << "| Mean stage: back substitution (s) | " << mean(t_back_sub) << " |\n";
    }

    std::cout << "Wrote detailed JSON: " << cfg.output_json << "\n";
    std::cout << "Wrote summary markdown: " << cfg.output_md << "\n";
    std::cout << "Solved cases: " << solved_cases << "/" << total_cases << "\n";
    std::cout << "Accepted FK-IK-FK cases: " << accepted_cases << "/" << total_cases << "\n";
    std::cout << "IK time mean/median/p95 (s): " << mean(t_ik) << " / " << median(t_ik) << " / "
              << quantile(t_ik, 0.95) << "\n";

    return 0;
}

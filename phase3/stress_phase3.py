#!/usr/bin/env python3
"""
Phase 3 stress test runner for numeric Python solver.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from ik_6r_general import forward_kinematics, solve_ik_6r_general


THIS_DIR = Path(__file__).resolve().parent


@dataclass
class SolutionMetrics:
    position_error: float
    orientation_error_rad: float
    transform_error: float


@dataclass
class CaseMetrics:
    case_index: int
    ik_time_sec: float
    num_solutions: int
    num_real_roots: int
    accepted_solutions: int
    best_position_error: float
    best_orientation_error_rad: float
    best_transform_error: float
    time_matrix_construction_sec: float
    time_poly_derivation_sec: float
    time_poly_solve_sec: float
    time_back_substitution_sec: float


def orientation_error_rad(r_target: np.ndarray, r_pred: np.ndarray) -> float:
    r_delta = r_target.T @ r_pred
    trace_val = float(np.trace(r_delta))
    cos_angle = max(-1.0, min(1.0, 0.5 * (trace_val - 1.0)))
    return float(math.acos(cos_angle))


def pose_errors(target: np.ndarray, pred: np.ndarray) -> SolutionMetrics:
    p_err = float(np.linalg.norm(target[:3, 3] - pred[:3, 3]))
    o_err = orientation_error_rad(target[:3, :3], pred[:3, :3])
    t_err = float(np.linalg.norm(target - pred, ord="fro"))
    return SolutionMetrics(position_error=p_err, orientation_error_rad=o_err, transform_error=t_err)


def random_dh(rng: np.random.Generator) -> np.ndarray:
    return np.column_stack(
        [
            rng.uniform(0.05, 0.60, size=6),
            rng.uniform(-math.pi, math.pi, size=6),
            rng.uniform(0.02, 0.50, size=6),
        ]
    ).astype(float)


def run_stress(
    num_dh_sets: int,
    samples_per_dh: int,
    seed: int,
    pos_threshold: float,
    ori_threshold: float,
    tf_threshold: float,
) -> tuple[dict[str, Any], list[CaseMetrics]]:
    rng = np.random.default_rng(seed)
    cases: list[CaseMetrics] = []
    total = num_dh_sets * samples_per_dh
    idx = 0

    for _ in range(num_dh_sets):
        dh = random_dh(rng)
        for _ in range(samples_per_dh):
            q_true = rng.uniform(-math.pi, math.pi, size=6).astype(float)
            target = forward_kinematics(dh, q_true)

            t0 = time.perf_counter()
            sols, diag = solve_ik_6r_general(
                dh,
                target,
                interpolation_grid=np.linspace(-6.0, 6.0, 41),
                root_solver="matrix_polynomial_pencil",
                matrix_pencil_degree=16,
                residual_tolerance=1e-4,
                return_diagnostics=True,
            )
            t_ik = float(time.perf_counter() - t0)

            accepted = 0
            best_pos = float("inf")
            best_ori = float("inf")
            best_tf = float("inf")
            for s in sols:
                fk = forward_kinematics(dh, np.array(s.q, dtype=float))
                errs = pose_errors(target, fk)
                best_pos = min(best_pos, errs.position_error)
                best_ori = min(best_ori, errs.orientation_error_rad)
                best_tf = min(best_tf, errs.transform_error)
                if (
                    errs.position_error <= pos_threshold
                    and errs.orientation_error_rad <= ori_threshold
                    and errs.transform_error <= tf_threshold
                ):
                    accepted += 1

            if not sols:
                best_pos = float("inf")
                best_ori = float("inf")
                best_tf = float("inf")

            cases.append(
                CaseMetrics(
                    case_index=idx,
                    ik_time_sec=t_ik,
                    num_solutions=len(sols),
                    num_real_roots=int(diag["num_polynomial_real_roots"]),
                    accepted_solutions=accepted,
                    best_position_error=best_pos,
                    best_orientation_error_rad=best_ori,
                    best_transform_error=best_tf,
                    time_matrix_construction_sec=float(diag.get("time_matrix_construction_sec", 0.0)),
                    time_poly_derivation_sec=float(diag.get("time_poly_derivation_sec", 0.0)),
                    time_poly_solve_sec=float(diag.get("time_poly_solve_sec", 0.0)),
                    time_back_substitution_sec=float(diag.get("time_back_substitution_sec", 0.0)),
                )
            )
            idx += 1
            if idx % 25 == 0 or idx == total:
                print(f"[progress] {idx}/{total} ({100.0 * idx / total:.1f}%)")

    ik_times = [c.ik_time_sec for c in cases]
    solved = sum(1 for c in cases if c.num_solutions > 0)
    accepted = sum(1 for c in cases if c.accepted_solutions > 0)

    summary: dict[str, Any] = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "seed": seed,
        "num_dh_sets": num_dh_sets,
        "samples_per_dh": samples_per_dh,
        "total_cases": total,
        "thresholds": {
            "position": pos_threshold,
            "orientation_rad": ori_threshold,
            "transform_fro": tf_threshold,
        },
        "cases_with_solution": solved,
        "cases_with_accepted_solution": accepted,
        "solve_success_rate": solved / total if total else float("nan"),
        "acceptance_rate": accepted / total if total else float("nan"),
        "ik_time_mean_sec": statistics.fmean(ik_times) if ik_times else float("nan"),
        "ik_time_median_sec": float(np.median(np.array(ik_times, dtype=float))) if ik_times else float("nan"),
        "ik_time_p95_sec": float(np.quantile(np.array(ik_times, dtype=float), 0.95)) if ik_times else float("nan"),
        "ik_time_max_sec": max(ik_times) if ik_times else float("nan"),
        "stage_matrix_construction_mean_sec": statistics.fmean(c.time_matrix_construction_sec for c in cases) if cases else float("nan"),
        "stage_poly_derivation_mean_sec": statistics.fmean(c.time_poly_derivation_sec for c in cases) if cases else float("nan"),
        "stage_poly_solve_mean_sec": statistics.fmean(c.time_poly_solve_sec for c in cases) if cases else float("nan"),
        "stage_back_substitution_mean_sec": statistics.fmean(c.time_back_substitution_sec for c in cases) if cases else float("nan"),
    }
    return summary, cases


def build_markdown(summary: dict[str, Any]) -> str:
    t = summary["thresholds"]
    lines: list[str] = []
    lines.append("# Phase 3 Stress Report")
    lines.append("")
    lines.append(f"- Generated at: `{summary['generated_at_utc']}`")
    lines.append(f"- Seed: `{summary['seed']}`")
    lines.append(f"- Total cases: `{summary['total_cases']}`")
    lines.append("")
    lines.append("## Thresholds")
    lines.append("")
    lines.append(f"- Position <= `{t['position']:.1e}`")
    lines.append(f"- Orientation(rad) <= `{t['orientation_rad']:.1e}`")
    lines.append(f"- Transform Frobenius <= `{t['transform_fro']:.1e}`")
    lines.append("")
    lines.append("## Aggregate Metrics")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("|---|---:|")
    lines.append(f"| Cases with >=1 IK solution | {summary['cases_with_solution']}/{summary['total_cases']} ({100.0*summary['solve_success_rate']:.2f}%) |")
    lines.append(f"| Cases with >=1 accepted FK-IK-FK solution | {summary['cases_with_accepted_solution']}/{summary['total_cases']} ({100.0*summary['acceptance_rate']:.2f}%) |")
    lines.append(f"| Mean IK time (s) | {summary['ik_time_mean_sec']:.6f} |")
    lines.append(f"| Median IK time (s) | {summary['ik_time_median_sec']:.6f} |")
    lines.append(f"| P95 IK time (s) | {summary['ik_time_p95_sec']:.6f} |")
    lines.append(f"| Max IK time (s) | {summary['ik_time_max_sec']:.6f} |")
    lines.append(f"| Mean stage: matrix construction (s) | {summary['stage_matrix_construction_mean_sec']:.6f} |")
    lines.append(f"| Mean stage: polynomial derivation (s) | {summary['stage_poly_derivation_mean_sec']:.6f} |")
    lines.append(f"| Mean stage: polynomial solve (s) | {summary['stage_poly_solve_mean_sec']:.6f} |")
    lines.append(f"| Mean stage: back substitution (s) | {summary['stage_back_substitution_mean_sec']:.6f} |")
    lines.append("")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--num-dh-sets", type=int, default=20)
    p.add_argument("--samples-per-dh", type=int, default=5)
    p.add_argument("--seed", type=int, default=20260303)
    p.add_argument("--pos-threshold", type=float, default=1e-4)
    p.add_argument("--ori-threshold", type=float, default=1e-4)
    p.add_argument("--tf-threshold", type=float, default=3e-4)
    p.add_argument("--out-json", type=Path, default=THIS_DIR / "phase3_stress_results.json")
    p.add_argument("--out-md", type=Path, default=THIS_DIR / "phase3_stress_report.md")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    summary, cases = run_stress(
        num_dh_sets=args.num_dh_sets,
        samples_per_dh=args.samples_per_dh,
        seed=args.seed,
        pos_threshold=args.pos_threshold,
        ori_threshold=args.ori_threshold,
        tf_threshold=args.tf_threshold,
    )

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(
        json.dumps({"summary": summary, "cases": [asdict(c) for c in cases]}, indent=2),
        encoding="utf-8",
    )
    args.out_md.write_text(build_markdown(summary), encoding="utf-8")
    print(f"[done] wrote {args.out_json}")
    print(f"[done] wrote {args.out_md}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

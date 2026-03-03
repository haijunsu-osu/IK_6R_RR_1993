#!/usr/bin/env python3
"""
Phase 3 deterministic FK-IK-FK validation for the numeric Python solver.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ik_6r_general import forward_kinematics, solve_ik_6r_general


THIS_DIR = Path(__file__).resolve().parent
REPORT_MD = THIS_DIR / "phase3_test_report.md"


@dataclass
class Thresholds:
    position: float = 1e-6
    orientation_rad: float = 1e-6
    transform_fro: float = 2e-6


def orientation_error_rad(r_target: np.ndarray, r_pred: np.ndarray) -> float:
    r_delta = r_target.T @ r_pred
    trace_val = float(np.trace(r_delta))
    cos_angle = max(-1.0, min(1.0, 0.5 * (trace_val - 1.0)))
    return float(math.acos(cos_angle))


def pose_errors(target_pose: np.ndarray, pred_pose: np.ndarray) -> tuple[float, float, float]:
    p_t = target_pose[:3, 3]
    p_p = pred_pose[:3, 3]
    r_t = target_pose[:3, :3]
    r_p = pred_pose[:3, :3]
    pos_err = float(np.linalg.norm(p_t - p_p))
    ori_err = orientation_error_rad(r_t, r_p)
    tf_err = float(np.linalg.norm(target_pose - pred_pose, ord="fro"))
    return pos_err, ori_err, tf_err


def example_dh() -> np.ndarray:
    return np.array(
        [
            [0.32, 0.70, 0.18],
            [0.25, -0.90, 0.21],
            [0.29, 0.80, 0.14],
            [0.22, -1.10, 0.19],
            [0.18, 0.60, 0.11],
            [0.15, -0.70, 0.17],
        ],
        dtype=float,
    )


def manseur_doty_case() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dh = np.array(
        [
            [0.3, math.pi / 2.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, math.pi / 2.0, 0.2],
            [1.5, 1.0, 0.0],
            [0.0, math.pi / 2.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=float,
    )
    q_true = np.array(
        [0.0, math.radians(107.458), math.radians(112.460), math.radians(-7.662), 0.0, 0.0],
        dtype=float,
    )
    target_published = np.array(
        [
            [-0.4655, -0.4822, 0.7421, -0.5377],
            [-0.6987, 0.7150, 0.0263, -1.1561],
            [-0.5433, -0.5063, -0.6697, 0.1895],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return dh, q_true, target_published


def evaluate_solver_case(
    name: str,
    dh: np.ndarray,
    target: np.ndarray,
    thresholds: Thresholds,
    **solver_kwargs: Any,
) -> dict[str, Any]:
    sols, diag = solve_ik_6r_general(dh, target, return_diagnostics=True, **solver_kwargs)
    accepted = 0
    best_pos = float("inf")
    best_ori = float("inf")
    best_tf = float("inf")

    for s in sols:
        fk = forward_kinematics(dh, np.array(s.q, dtype=float))
        pos_err, ori_err, tf_err = pose_errors(target, fk)
        best_pos = min(best_pos, pos_err)
        best_ori = min(best_ori, ori_err)
        best_tf = min(best_tf, tf_err)
        if (
            pos_err <= thresholds.position
            and ori_err <= thresholds.orientation_rad
            and tf_err <= thresholds.transform_fro
        ):
            accepted += 1

    if not sols:
        best_pos, best_ori, best_tf = float("inf"), float("inf"), float("inf")

    return {
        "name": name,
        "diag": diag,
        "num_solutions": len(sols),
        "accepted_solutions": accepted,
        "best_position_error": best_pos,
        "best_orientation_error_rad": best_ori,
        "best_transform_error": best_tf,
    }


def main() -> int:
    th = Thresholds()
    rows: list[dict[str, Any]] = []

    dh = example_dh()
    q_true = np.array([0.60, -1.00, 0.90, -0.80, 1.20, -0.40], dtype=float)
    target = forward_kinematics(dh, q_true)
    rows.append(
        evaluate_solver_case(
            "seeded_random_like_case",
            dh,
            target,
            th,
            interpolation_grid=np.linspace(-6.0, 6.0, 41),
            root_solver="matrix_polynomial_pencil",
            matrix_pencil_degree=16,
            residual_tolerance=1e-4,
        )
    )

    dh16, q16, target_published = manseur_doty_case()
    target16_fk = forward_kinematics(dh16, q16)
    rows.append(
        evaluate_solver_case(
            "manseur_doty_16_real_case",
            dh16,
            target16_fk,
            th,
            interpolation_grid=np.linspace(-12.0, 12.0, 41),
            root_solver="matrix_polynomial_pencil",
            matrix_pencil_degree=16,
            residual_tolerance=1e-4,
        )
    )

    published_diff = float(np.max(np.abs(target16_fk - target_published)))
    row_random = next(r for r in rows if r["name"] == "seeded_random_like_case")
    row_16 = next(r for r in rows if r["name"] == "manseur_doty_16_real_case")

    pass_random = row_random["accepted_solutions"] >= 1
    pass_16 = (
        row_16["accepted_solutions"] >= 16
        and int(row_16["diag"]["num_polynomial_real_roots"]) == 16
        and int(row_16["diag"]["num_candidate_x3_roots"]) == 16
    )
    pass_published = published_diff < 5e-4
    overall_pass = pass_random and pass_16 and pass_published

    lines: list[str] = []
    lines.append("# Phase 3 Test Report")
    lines.append("")
    lines.append("## Thresholds")
    lines.append("")
    lines.append(f"- Position error norm <= `{th.position:.1e}`")
    lines.append(f"- Orientation error (rad) <= `{th.orientation_rad:.1e}`")
    lines.append(f"- Full transform Frobenius norm <= `{th.transform_fro:.1e}`")
    lines.append("")
    lines.append("## FK-IK-FK Results")
    lines.append("")
    lines.append("| Case | #Solutions | #Accepted | Best Pos Err | Best Ori Err (rad) | Best TF Err | #Real Roots |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        diag = r["diag"]
        lines.append(
            f"| {r['name']} | {r['num_solutions']} | {r['accepted_solutions']} | "
            f"{r['best_position_error']:.3e} | {r['best_orientation_error_rad']:.3e} | "
            f"{r['best_transform_error']:.3e} | {int(diag['num_polynomial_real_roots'])} |"
        )
    lines.append("")
    lines.append("## Mandatory 16-Solution Reference Check")
    lines.append("")
    lines.append(f"- Published target max abs diff vs FK(q_true): `{published_diff:.3e}` (must be < `5e-4`)")
    lines.append("")
    lines.append("## Result")
    lines.append("")
    lines.append(f"- Overall pass: `{overall_pass}`")
    lines.append(f"- Random case pass (`accepted >= 1`): `{pass_random}`")
    lines.append(f"- 16-real case pass (`accepted >= 16` and roots=16): `{pass_16}`")
    lines.append(f"- Published pose consistency pass: `{pass_published}`")

    REPORT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[done] wrote {REPORT_MD}")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())

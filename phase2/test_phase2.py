#!/usr/bin/env python3
"""
Phase 2 test harness:
1) run Mathematica symbolic stress checks
2) export solved IK branches from the symbolic solver
3) verify FK-IK-FK residual thresholds in Python
4) emit phase2_test_report.md
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent

STRESS_JSON = THIS_DIR / "phase2_stress_results.json"
FKIK_JSON = THIS_DIR / "phase2_fkik_cases.json"
REPORT_MD = THIS_DIR / "phase2_test_report.md"


@dataclass
class Thresholds:
    position: float = 1e-4
    orientation_rad: float = 1e-4
    transform_fro: float = 3e-4


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )


def _aiv(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array(
        [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def _ais(a: float, alpha: float, d: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [[1.0, 0.0, 0.0, a], [0.0, ca, -sa, 0.0], [0.0, sa, ca, d], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def _fk(dh: np.ndarray, q: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    for i in range(6):
        t = t @ (_aiv(float(q[i])) @ _ais(float(dh[i, 0]), float(dh[i, 1]), float(dh[i, 2])))
    return t


def _orientation_err(r_target: np.ndarray, r_pred: np.ndarray) -> float:
    r_delta = r_target.T @ r_pred
    tr = float(np.trace(r_delta))
    c = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    return float(math.acos(c))


def _validate_case(case: dict[str, Any], th: Thresholds) -> dict[str, Any]:
    dh = np.array(case["dh"], dtype=float)
    target = np.array(case["target"], dtype=float)
    sols = [np.array(s, dtype=float) for s in case["solutions"]]

    accepted = 0
    pos_max = 0.0
    ori_max = 0.0
    tf_max = 0.0
    best_pos = float("inf")
    best_ori = float("inf")
    best_tf = float("inf")

    for q in sols:
        pred = _fk(dh, q)
        pos_err = float(np.linalg.norm(pred[:3, 3] - target[:3, 3]))
        ori_err = _orientation_err(target[:3, :3], pred[:3, :3])
        tf_err = float(np.linalg.norm(pred - target, ord="fro"))

        pos_max = max(pos_max, pos_err)
        ori_max = max(ori_max, ori_err)
        tf_max = max(tf_max, tf_err)
        best_pos = min(best_pos, pos_err)
        best_ori = min(best_ori, ori_err)
        best_tf = min(best_tf, tf_err)

        if pos_err <= th.position and ori_err <= th.orientation_rad and tf_err <= th.transform_fro:
            accepted += 1

    if not sols:
        best_pos = float("inf")
        best_ori = float("inf")
        best_tf = float("inf")

    return {
        "name": case["name"],
        "num_solutions": len(sols),
        "accepted_solutions": accepted,
        "best_position_error": best_pos,
        "best_orientation_error_rad": best_ori,
        "best_transform_fro_error": best_tf,
        "worst_position_error": pos_max,
        "worst_orientation_error_rad": ori_max,
        "worst_transform_fro_error": tf_max,
    }


def main() -> int:
    # 1) symbolic consistency stress run
    stress_run = _run(["wolframscript", "-file", str(THIS_DIR / "run_symbolic_stress.wls")])
    if stress_run.returncode != 0:
        print(stress_run.stdout)
        print(stress_run.stderr, file=sys.stderr)
        print("[error] Phase 2 symbolic stress run failed.", file=sys.stderr)
        return 1

    # 2) export solved FK-IK-FK validation cases
    fkik_run = _run(["wolframscript", "-file", str(THIS_DIR / "export_fkik_validation_cases.wls")])
    if fkik_run.returncode != 0:
        print(fkik_run.stdout)
        print(fkik_run.stderr, file=sys.stderr)
        print("[error] Phase 2 FK-IK-FK case export failed.", file=sys.stderr)
        return 1

    stress = json.loads(STRESS_JSON.read_text(encoding="utf-8"))
    fkik_payload = json.loads(FKIK_JSON.read_text(encoding="utf-8"))

    th = Thresholds()
    case_results = [_validate_case(c, th) for c in fkik_payload["cases"]]

    random_case = next(c for c in case_results if c["name"] == "random_seed_20260317")
    case16 = next(c for c in case_results if c["name"] == "manseur_doty_16_real_case")

    pass_random = random_case["accepted_solutions"] >= 1
    pass_16 = case16["accepted_solutions"] >= 15
    pass_stress = (
        int(stress["num_random_passed"]) == int(stress["num_random_cases"])
        and bool(stress["case_16_real_solution"]["pass"])
    )
    overall_pass = pass_random and pass_16 and pass_stress

    lines: list[str] = []
    lines.append("# Phase 2 Test Report")
    lines.append("")
    lines.append("## Thresholds")
    lines.append("")
    lines.append(f"- Position error norm <= `{th.position:.1e}`")
    lines.append(f"- Orientation error (rad) <= `{th.orientation_rad:.1e}`")
    lines.append(f"- Full transform Frobenius norm <= `{th.transform_fro:.1e}`")
    lines.append("")
    lines.append("## Symbolic Stress Checks")
    lines.append("")
    lines.append(f"- Random substitution cases passed: `{stress['num_random_passed']}/{stress['num_random_cases']}`")
    lines.append(
        "- 16-real-solution symbolic case: "
        f"roots=`{stress['case_16_real_solution']['num_real_x3_roots']}`, "
        f"solutions=`{stress['case_16_real_solution']['num_solutions']}`, "
        f"pass=`{stress['case_16_real_solution']['pass']}`"
    )
    lines.append("")
    lines.append("## FK-IK-FK Validation")
    lines.append("")
    lines.append("| Case | #Solutions | #Accepted | Best Pos Err | Best Ori Err (rad) | Best TF Err |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for c in case_results:
        lines.append(
            f"| {c['name']} | {c['num_solutions']} | {c['accepted_solutions']} | "
            f"{c['best_position_error']:.3e} | {c['best_orientation_error_rad']:.3e} | "
            f"{c['best_transform_fro_error']:.3e} |"
        )
    lines.append("")
    lines.append("## Result")
    lines.append("")
    lines.append(f"- Overall pass: `{overall_pass}`")
    lines.append(f"- Random case pass criterion (`accepted >= 1`): `{pass_random}`")
    lines.append(f"- 16-root case pass criterion (`accepted >= 15`): `{pass_16}`")
    lines.append(f"- Symbolic stress pass criterion: `{pass_stress}`")

    REPORT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[done] wrote {REPORT_MD}")
    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())

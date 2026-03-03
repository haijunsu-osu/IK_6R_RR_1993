#!/usr/bin/env python3
"""
Phase 4 runner:
1) build C++ solver
2) run deterministic C++ test executable
3) run C++ random stress benchmark
4) compare C++ aggregate metrics against Phase 3 Python stress results
"""

from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PHASE4 = ROOT / "phase4"
BUILD = PHASE4 / "build_release"

CPP_STRESS_JSON = PHASE4 / "phase4_stress_results_cpp.json"
CPP_STRESS_MD = PHASE4 / "phase4_stress_report.md"
PHASE4_TEST_MD = PHASE4 / "phase4_test_report.md"
CPP_VS_PY_MD = PHASE4 / "python_vs_cpp_comparison_summary.md"

PHASE3_STRESS_JSON = ROOT / "phase3" / "phase3_stress_results.json"


def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        capture_output=True,
        check=False,
    )


def ensure_ok(proc: subprocess.CompletedProcess[str], desc: str) -> None:
    if proc.returncode == 0:
        return
    sys.stderr.write(f"[error] {desc} failed (exit={proc.returncode})\n")
    if proc.stdout:
        sys.stderr.write(proc.stdout + "\n")
    if proc.stderr:
        sys.stderr.write(proc.stderr + "\n")
    raise SystemExit(proc.returncode)


def pct(num: float, den: float) -> float:
    return 100.0 * num / den if den else float("nan")


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)

    cfg = run(
        ["cmake", "-S", str(PHASE4), "-B", str(BUILD), "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release"],
        cwd=ROOT,
    )
    ensure_ok(cfg, "cmake configure")

    bld = run(["cmake", "--build", str(BUILD), "-j", "8"], cwd=ROOT)
    ensure_ok(bld, "cmake build")

    exe_test = BUILD / "test_ik_6r_general.exe"
    exe_test16 = BUILD / "test_manseur_doty_16.exe"
    exe_bench = BUILD / "benchmark_random.exe"
    if not exe_test.exists() or not exe_test16.exists() or not exe_bench.exists():
        raise SystemExit("[error] expected test/benchmark executables were not generated.")

    test_run = run([str(exe_test)], cwd=ROOT)
    ensure_ok(test_run, "phase4 deterministic C++ test")
    test16_run = run([str(exe_test16)], cwd=ROOT)
    ensure_ok(test16_run, "phase4 Manseur-Doty 16-root C++ test")

    bench_run = run(
        [
            str(exe_bench),
            "--num-dh-sets",
            "20",
            "--samples-per-dh",
            "5",
            "--seed",
            "20260303",
            "--output-json",
            str(CPP_STRESS_JSON),
            "--output-md",
            str(CPP_STRESS_MD),
        ],
        cwd=ROOT,
    )
    ensure_ok(bench_run, "phase4 C++ stress benchmark")

    cpp_json = json.loads(CPP_STRESS_JSON.read_text(encoding="utf-8"))
    cpp_summary = cpp_json["summary"]

    py_json = json.loads(PHASE3_STRESS_JSON.read_text(encoding="utf-8"))
    py_summary = py_json["summary"]

    py_total = float(py_summary["total_cases"])
    py_solved = float(py_summary["cases_with_solution"])
    py_accepted = float(py_summary["cases_with_accepted_solution"])
    py_solve_rate = py_summary.get("solve_success_rate", py_solved / py_total if py_total else float("nan"))
    py_accept_rate = py_summary.get("acceptance_rate", py_accepted / py_total if py_total else float("nan"))

    cpp_total = float(cpp_summary["total_cases"])
    cpp_solved = float(cpp_summary["cases_with_solution"])
    cpp_solve_rate = cpp_solved / cpp_total if cpp_total else float("nan")
    cpp_accepted = float(cpp_summary.get("cases_with_accepted_solution", float("nan")))
    cpp_accept_rate = float(cpp_summary.get("acceptance_rate", float("nan")))

    py_mean = float(py_summary["ik_time_mean_sec"])
    cpp_mean = float(cpp_summary["ik_time_mean_sec"])
    speedup = py_mean / cpp_mean if cpp_mean > 0 else float("nan")

    # Phase 4 execution report
    lines_test: list[str] = []
    lines_test.append("# Phase 4 Test Report")
    lines_test.append("")
    lines_test.append(f"- Generated at: `{datetime.now(timezone.utc).isoformat()}`")
    lines_test.append("- Deterministic executable: `test_ik_6r_general.exe`")
    lines_test.append("- Random stress executable: `benchmark_random.exe`")
    lines_test.append("")
    lines_test.append("## Build")
    lines_test.append("")
    lines_test.append("```text")
    lines_test.append("cmake -S phase4 -B phase4/build_release -G Ninja -DCMAKE_BUILD_TYPE=Release")
    lines_test.append("cmake --build phase4/build_release -j 8")
    lines_test.append("```")
    lines_test.append("")
    lines_test.append("## Deterministic Test Output")
    lines_test.append("")
    lines_test.append("```text")
    lines_test.append((test_run.stdout or "").strip() or "[no stdout]")
    if test_run.stderr:
        lines_test.append(test_run.stderr.strip())
    lines_test.append((test16_run.stdout or "").strip() or "[no stdout]")
    if test16_run.stderr:
        lines_test.append(test16_run.stderr.strip())
    lines_test.append("```")
    lines_test.append("")
    lines_test.append("## Stress Summary (C++)")
    lines_test.append("")
    lines_test.append(f"- Cases with solution: `{int(cpp_solved)}/{int(cpp_total)}` ({pct(cpp_solved, cpp_total):.2f}%)")
    if not (cpp_accepted != cpp_accepted):  # NaN check
        lines_test.append(
            f"- Cases with accepted FK-IK-FK solution: `{int(cpp_accepted)}/{int(cpp_total)}` "
            f"({100.0*cpp_accept_rate:.2f}%)"
        )
    lines_test.append(f"- Mean IK time: `{cpp_summary['ik_time_mean_sec']:.6f}` s")
    lines_test.append(f"- Median IK time: `{cpp_summary['ik_time_median_sec']:.6f}` s")
    lines_test.append(f"- P95 IK time: `{cpp_summary['ik_time_p95_sec']:.6f}` s")
    lines_test.append("")
    PHASE4_TEST_MD.write_text("\n".join(lines_test) + "\n", encoding="utf-8")

    # Python-vs-C++ comparison report
    lines_cmp: list[str] = []
    lines_cmp.append("# Python vs C++ Comparison")
    lines_cmp.append("")
    lines_cmp.append("| Metric | Python (Phase 3) | C++ (Phase 4) |")
    lines_cmp.append("|---|---:|---:|")
    lines_cmp.append(f"| Total cases | {int(py_total)} | {int(cpp_total)} |")
    lines_cmp.append(f"| Cases with >=1 IK solution | {int(py_solved)} ({100.0*py_solve_rate:.2f}%) | {int(cpp_solved)} ({100.0*cpp_solve_rate:.2f}%) |")
    if cpp_accepted == cpp_accepted:
        lines_cmp.append(
            f"| Cases with >=1 accepted FK-IK-FK solution | "
            f"{int(py_accepted)} ({100.0*py_accept_rate:.2f}%) | "
            f"{int(cpp_accepted)} ({100.0*cpp_accept_rate:.2f}%) |"
        )
    else:
        lines_cmp.append(f"| Cases with >=1 accepted FK-IK-FK solution | {int(py_accepted)} ({100.0*py_accept_rate:.2f}%) | NA |")
    lines_cmp.append(f"| Mean IK time (s) | {py_mean:.6f} | {cpp_mean:.6f} |")
    lines_cmp.append(f"| Median IK time (s) | {float(py_summary['ik_time_median_sec']):.6f} | {float(cpp_summary['ik_time_median_sec']):.6f} |")
    lines_cmp.append(f"| P95 IK time (s) | {float(py_summary['ik_time_p95_sec']):.6f} | {float(cpp_summary['ik_time_p95_sec']):.6f} |")
    lines_cmp.append(f"| Speedup (Python mean / C++ mean) | {speedup:.2f}x | - |")
    lines_cmp.append("")
    lines_cmp.append("Notes:")
    lines_cmp.append("- Both stress runs use position/orientation/transform thresholds for acceptance.")
    CPP_VS_PY_MD.write_text("\n".join(lines_cmp) + "\n", encoding="utf-8")

    print(f"[done] wrote {PHASE4_TEST_MD}")
    print(f"[done] wrote {CPP_STRESS_MD}")
    print(f"[done] wrote {CPP_VS_PY_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

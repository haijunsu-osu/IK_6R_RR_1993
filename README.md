# RR1993 General 6R IK Benchmark (Prompt Workspace)

Prof. Hai-Jun Su's lab project page: [AI Kinematics Tutorial](https://su-idr-lab.github.io/projects/msr_2026_ai_kinematics_tutorial/index.html)

This repository contains a complete 4-phase reconstruction of a general 6R inverse kinematics (IK) pipeline from:

- [2026 ASME IDETC/CIE preprint](2026_ASME_IDETC_AGI.pdf)
- `RaghavanRoth6R1993.pdf` (resultant/elimination formulation)
- `MCtra94.pdf` (matrix-polynomial / generalized-eigenvalue root strategy)
- `A_Robot_Manipulator_With_16_Real_Inverse_Kinematic.pdf` (mandatory validation case)

The workflow is organized into `phase1` to `phase4`, with consolidated reporting in `reports/final_report.md`.

## Provenance

The files under `phase1`, `phase2`, `phase3`, `phase4`, and `reports` were created from a single-shot prompt in Codex 5.3:

`read this prompt file (prompt_rr1993_general_6r_ik_detailed.md) and proceed the task specified in this file`

Execution context:
- Codex 5.3 (`Extra High` option)
- VS Code environment
- Mathematica 14.3 installed on the local computer

## Repository Layout

- `phase1/`: PDF extraction + OCR fallback
- `phase2/`: symbolic derivation (Wolfram) + phase-2 validation
- `phase3/`: numeric Python solver + deterministic/stress tests
- `phase4/`: C++ solver + deterministic/stress tests + Python-vs-C++ comparison
- `reports/`: final integrated report

## Prerequisites

### Python

- Python 3.10+ recommended
- Install dependencies:

```powershell
python -m pip install -r requirements.txt
python -m pip install pypdf scipy
```

Notes:
- `phase1` uses `pypdf` directly.
- OCR runtime with `pytesseract` additionally requires Tesseract installed on system PATH.
- If Tesseract is unavailable, `phase1` falls back to sidecar OCR text where available.

### Wolfram (Phase 2)

- `wolframscript` must be installed and available in PATH.

### C++ toolchain (Phase 4)

- CMake 3.20+
- Ninja
- C++20 compiler (MSVC/clang/gcc)
- Eigen headers

Default Eigen path is configured in `phase4/CMakeLists.txt` as:

`C:/Program Files/MATLAB/R2025b/toolbox/shared/robotics/externalDependency/eigen/include/eigen3`

Override it with:

```powershell
cmake -S phase4 -B phase4/build_release -G Ninja -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR="C:/path/to/eigen3"
```

## Quick Start (Run Everything)

From repository root:

```powershell
python phase1/extract_pdf_text_ocr.py
python phase2/test_phase2.py
python phase3/test_phase3.py
python phase3/stress_phase3.py
python phase4/test_phase4.py
```

Final summary:

```text
reports/final_report.md
```

## Re-test The Single-Shot Prompt (Clean Reproduction)

If you want to test the prompt workflow from scratch in this workspace:

1. Keep these source files in the workspace root:
- `RaghavanRoth6R1993.pdf`
- `MCtra94.pdf`
- `A_Robot_Manipulator_With_16_Real_Inverse_Kinematic.pdf`
- `prompt_rr1993_general_6r_ik_detailed.md` (your prompt file; if you use a different name such as `prompt.md`, keep that file present instead)

2. Remove generated output folders:

```powershell
Remove-Item -Recurse -Force phase1, phase2, phase3, phase4, reports
```

3. In Codex (VS Code), submit this single prompt:

```text
read this prompt file (prompt_rr1993_general_6r_ik_detailed.md) and proceed the task specified in this file
```

4. Expected result:
- Codex recreates `phase1`, `phase2`, `phase3`, `phase4`, and `reports` with scripts, tests, and reports.

## Phase-by-Phase Usage

### Phase 1: PDF Text/OCR Extraction

Run:

```powershell
python phase1/extract_pdf_text_ocr.py
```

Outputs:
- `phase1/artifacts/*_extracted.md`
- `phase1/artifacts/*_equations.md`
- `phase1/artifacts/*_page_trace.json`
- `phase1/artifacts/phase1_extraction_summary.json`

### Phase 2: Symbolic Derivation + Validation

Main derivation script:
- `phase2/rr1993_symbolic_derivation.wls`

Run validation:

```powershell
python phase2/test_phase2.py
```

Outputs:
- `phase2/phase2_test_report.md`
- `phase2/phase2_stress_results.json`
- `phase2/phase2_fkik_cases.json`

### Phase 3: Numeric Python Solver

Core solver:
- `phase3/ik_6r_general.py`

Deterministic tests:

```powershell
python phase3/test_phase3.py
```

Stress tests:

```powershell
python phase3/stress_phase3.py
```

Outputs:
- `phase3/phase3_test_report.md`
- `phase3/phase3_stress_report.md`
- `phase3/phase3_stress_results.json`

### Phase 4: C++ Translation + Validation

Automated build/test/benchmark:

```powershell
python phase4/test_phase4.py
```

This runs:
- `test_ik_6r_general.exe`
- `test_manseur_doty_16.exe` (mandatory 16-real-root case check)
- `benchmark_random.exe`

Outputs:
- `phase4/phase4_test_report.md`
- `phase4/phase4_stress_report.md`
- `phase4/phase4_stress_results_cpp.json`
- `phase4/python_vs_cpp_comparison_summary.md`

## Key Validation Criteria

FK-IK-FK checks use explicit thresholds for:
- position error norm
- orientation error (radians)
- full transform matrix discrepancy (Frobenius norm)

Exact thresholds per phase are documented in:
- `phase2/phase2_test_report.md`
- `phase3/phase3_test_report.md`
- `phase3/phase3_stress_report.md`
- `phase4/phase4_stress_report.md`

## Mandatory Reference Case

The published `A Robot Manipulator With 16 Real Inverse Kinematic Solution Sets` case is included as a required validation target in:
- Phase 2 symbolic checks
- Phase 3 Python deterministic tests
- Phase 4 C++ deterministic tests

## Main Results

See:

- `reports/final_report.md`

It summarizes:
- methods used in each phase
- thresholds and pass/fail criteria
- stress statistics
- 16-real-solution case behavior
- Python-vs-C++ comparison

## Citation

> **Preprint:** [2026 ASME IDETC/CIE preprint](2026_ASME_IDETC_AGI.pdf)
>
> HJ Su, JM McCarthy, Where we are in the path towards AGI in kinematics Proceedings of the ASME 2026 International Design Engineering Technical Conferences and Computers and Information in Engineering Conference (IDETC/CIE 2026), Houston, TX, August 23–26, 2026, Paper No. DETC2026-193961.
>
> **Track:** Mechanisms and Robotics Conference

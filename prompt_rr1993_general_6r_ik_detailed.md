# Prompt: Build a General 6R IK Solver from Three Reference Papers (New Session)

## (1) Context Block
You are starting a **brand-new session** as an expert in robot kinematics, symbolic elimination, and numerical scientific computing.  
Assume you have **no access to any existing project files or prior code**.  
You are only given these three reference PDFs:

1. `RaghavanRoth6R1993.pdf`  
   - Primary derivation of general 6R IK via elimination/resultants.
2. `MCtra94.pdf`  
   - Companion-matrix / generalized-eigenvalue strategy for solving the final polynomial system.
3. `A_Robot_Manipulator_With_16_Real_Inverse_Kinematic.pdf`  
   - Numerical 6R example with 16 real IK solutions (validation target case).

Your job is to reconstruct the full pipeline from these references and deliver reproducible scripts and test evidence.

---

## (2) Task Block
Implement the project in **4 phases**:

### Phase 1: PDF text extraction with OCR
Write a Python script that extracts text and equations from the three PDFs.  
If direct text extraction fails for scanned pages, automatically use OCR.  
Store extracted artifacts in readable text/markdown files for traceability.

### Phase 2: Symbolic derivation (CAS)
Using CAS (Mathematica/Wolfram), derive the elimination steps for general 6R IK from RR1993, including:
- the equation setup,
- elimination sequence,
- final polynomial/root-solving form compatible with MC94 strategy.

Deliver a `.wls` script that performs the symbolic derivation and outputs intermediate checks.

### Phase 3: Numerical solver in Python
Implement a numeric-only Python solver (runtime must not rely on symbolic tools) that:
- computes IK solutions for a general 6R chain,
- uses the elimination-derived structure from Phase 2,
- uses numerical polynomial/root solving aligned with MC94 generalized-eigen approach,
- returns all valid real IK branches when present.

### Phase 4: Translate solver to C++
Translate the Phase 3 numeric solver into C++ with equivalent logic, tolerances, and output behavior.

---

## (3) Requirements Block

### A. Testing requirements for Phases 2-4
For each of Phases 2, 3, and 4, provide a dedicated test script that validates the FK-IK-FK cycle:

1. Choose or generate joint angles `q_true`.
2. Use forward kinematics (FK) to compute target pose `T_target`.
3. Run IK on `T_target` to recover candidate joint solutions.
4. Re-apply FK to each recovered solution and compare with `T_target`.
5. Accept solutions only when the final pose discrepancy is below a specified threshold.

Use explicit thresholds for:
- position error norm,
- orientation error (or equivalent rotation discrepancy),
- optional full transform matrix norm.

Set thresholds to be sufficiently strict for reliable numerical correctness, and report them clearly.

### B. Stress testing requirements
Run multiple stress tests in every phase:

- **Phase 2 stress tests**: symbolic derivation consistency checks across multiple random numeric substitutions.
- **Phase 3 stress tests**: multiple random 6R instances and targets; report solve success rate, residuals, and runtime stats.
- **Phase 4 stress tests**: same stress regime in C++; compare key metrics against Python.

Also include the published 16-real-solution example from  
`A_Robot_Manipulator_With_16_Real_Inverse_Kinematic.pdf` as a mandatory validation case.

### C. Numerical and engineering constraints
- Keep the solver general in DH parameters and target pose.
- Runtime solver (Phases 3-4) must be numeric-only.
- Use deterministic random seeds for reproducibility.
- Record both per-case and aggregate statistics.
- Clearly separate core solver runtime from non-solver overhead when reporting timing.

---

## (4) Deliverables Block
Save all scripts in the current workspace and provide:

1. **Phase 1 artifacts**
   - Python OCR/text extraction script (`.py`)
   - extracted text/equation artifacts from each reference PDF

2. **Phase 2 artifacts**
   - Mathematica symbolic derivation script (`.wls`)
   - Phase 2 test script + execution report

3. **Phase 3 artifacts**
   - Python numerical IK solver (`.py`)
   - Phase 3 test script + execution report
   - stress-test report

4. **Phase 4 artifacts**
   - C++ IK solver implementation (source + headers)
   - Phase 4 test script/executable + execution report
   - stress-test report and Python-vs-C++ comparison summary

5. **Final report**
   - concise markdown report summarizing:
     - method used in each phase,
     - thresholds and validation criteria,
     - pass/fail outcomes,
     - stress-test statistics,
     - 16-real-solution case results.


# Phase 4 Test Report

- Generated at: `2026-03-03T02:25:46.333654+00:00`
- Deterministic executable: `test_ik_6r_general.exe`
- Random stress executable: `benchmark_random.exe`

## Build

```text
cmake -S phase4 -B phase4/build_release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build phase4/build_release -j 8
```

## Deterministic Test Output

```text
test_ik_6r_general passed. solutions=2
test_manseur_doty_16 passed. solutions=15, accepted=15, real_roots=16
```

## Stress Summary (C++)

- Cases with solution: `100/100` (100.00%)
- Cases with accepted FK-IK-FK solution: `100/100` (100.00%)
- Mean IK time: `0.001585` s
- Median IK time: `0.001570` s
- P95 IK time: `0.002046` s


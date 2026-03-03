# Phase 3 Stress Report

- Generated at: `2026-03-03T02:17:42.102671+00:00`
- Seed: `20260303`
- Total cases: `100`

## Thresholds

- Position <= `1.0e-04`
- Orientation(rad) <= `1.0e-04`
- Transform Frobenius <= `3.0e-04`

## Aggregate Metrics

| Metric | Value |
|---|---:|
| Cases with >=1 IK solution | 99/100 (99.00%) |
| Cases with >=1 accepted FK-IK-FK solution | 99/100 (99.00%) |
| Mean IK time (s) | 0.174350 |
| Median IK time (s) | 0.172462 |
| P95 IK time (s) | 0.204367 |
| Max IK time (s) | 0.764049 |
| Mean stage: matrix construction (s) | 0.007888 |
| Mean stage: polynomial derivation (s) | 0.000079 |
| Mean stage: polynomial solve (s) | 0.044056 |
| Mean stage: back substitution (s) | 0.011582 |


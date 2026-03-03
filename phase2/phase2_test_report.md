# Phase 2 Test Report

## Thresholds

- Position error norm <= `1.0e-04`
- Orientation error (rad) <= `1.0e-04`
- Full transform Frobenius norm <= `3.0e-04`

## Symbolic Stress Checks

- Random substitution cases passed: `8/8`
- 16-real-solution symbolic case: roots=`16`, solutions=`15`, pass=`True`

## FK-IK-FK Validation

| Case | #Solutions | #Accepted | Best Pos Err | Best Ori Err (rad) | Best TF Err |
|---|---:|---:|---:|---:|---:|
| random_seed_20260317 | 4 | 4 | 1.064e-14 | 0.000e+00 | 1.676e-14 |
| manseur_doty_16_real_case | 15 | 15 | 1.337e-15 | 0.000e+00 | 2.327e-15 |

## Result

- Overall pass: `True`
- Random case pass criterion (`accepted >= 1`): `True`
- 16-root case pass criterion (`accepted >= 15`): `True`
- Symbolic stress pass criterion: `True`

# Phase 3 Test Report

## Thresholds

- Position error norm <= `1.0e-06`
- Orientation error (rad) <= `1.0e-06`
- Full transform Frobenius norm <= `2.0e-06`

## FK-IK-FK Results

| Case | #Solutions | #Accepted | Best Pos Err | Best Ori Err (rad) | Best TF Err | #Real Roots |
|---|---:|---:|---:|---:|---:|---:|
| seeded_random_like_case | 2 | 2 | 1.605e-15 | 0.000e+00 | 5.280e-15 | 2 |
| manseur_doty_16_real_case | 16 | 16 | 2.685e-15 | 0.000e+00 | 4.244e-15 | 16 |

## Mandatory 16-Solution Reference Check

- Published target max abs diff vs FK(q_true): `4.898e-05` (must be < `5e-4`)

## Result

- Overall pass: `True`
- Random case pass (`accepted >= 1`): `True`
- 16-real case pass (`accepted >= 16` and roots=16): `True`
- Published pose consistency pass: `True`

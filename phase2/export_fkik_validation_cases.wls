(* Export solved IK branches from the symbolic solver for FK-IK-FK validation in Python. *)

System`$Phase2ScriptDir = If[StringQ[$InputFileName] && StringLength[$InputFileName] > 0,
  DirectoryName[$InputFileName],
  Directory[]
];
$RR1993NoAutoRun = True;
Get[FileNameJoin[{System`$Phase2ScriptDir, "rr1993_symbolic_derivation.wls"}]];
scriptDir = System`$Phase2ScriptDir;

scriptArgs = If[Length[$ScriptCommandLine] >= 4, $ScriptCommandLine[[4 ;;]], {}];
outPath = If[Length[scriptArgs] >= 1,
  scriptArgs[[1]],
  FileNameJoin[{scriptDir, "phase2_fkik_cases.json"}]
];

caseRandom = Module[
  {ex, dh, qTrue, target, res},
  ex = GenerateRandomGeneralCase[20260317];
  dh = N[ex["DH"], 20];
  qTrue = N[ex["QTrue"], 20];
  target = N[ForwardKinematics[dh, qTrue], 20];
  res = SolveIKRR1993[
    dh,
    target,
    RRSampleCount -> 45,
    RRInterpolationGrid -> N[Range[-6, 6, 0.5]],
    RRResidualTolerance -> 1.*^-4
  ];
  <|
    "name" -> "random_seed_20260317",
    "dh" -> dh,
    "target" -> target,
    "q_true" -> qTrue,
    "solutions" -> (N[#["q"], 20] & /@ res["Solutions"]),
    "solver_residuals" -> (N[#["residual"], 20] & /@ res["Solutions"])
  |>
];

case16 = Module[
  {dh, qTrue, target, res},
  dh = N[{
    {0.3, Pi/2, 0.0},
    {1.0, 1.0, 0.0},
    {0.0, Pi/2, 0.2},
    {1.5, 1.0, 0.0},
    {0.0, Pi/2, 0.0},
    {0.0, 1.0, 0.0}
  }, 20];
  qTrue = N[{0.0, 107.458 Degree, 112.460 Degree, -7.662 Degree, 0.0, 0.0}, 20];
  target = N[ForwardKinematics[dh, qTrue], 20];
  res = SolveIKRR1993[
    dh,
    target,
    RRSampleCount -> 50,
    RRInterpolationGrid -> N[Range[-8, 8, 0.5]],
    RRResidualTolerance -> 1.*^-4
  ];
  <|
    "name" -> "manseur_doty_16_real_case",
    "dh" -> dh,
    "target" -> target,
    "q_true" -> qTrue,
    "solutions" -> (N[#["q"], 20] & /@ res["Solutions"]),
    "solver_residuals" -> (N[#["residual"], 20] & /@ res["Solutions"]),
    "num_real_x3_roots" -> Length[res["RealX3Roots"]]
  |>
];

payload = <|
  "cases" -> {caseRandom, case16}
|>;

Export[outPath, payload, "JSON"];
Print["Wrote FK-IK-FK case export: ", outPath];
Exit[0];

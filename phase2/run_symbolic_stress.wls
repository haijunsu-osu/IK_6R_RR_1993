(* Phase 2 stress/consistency test runner for RR1993 symbolic elimination script. *)

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
  FileNameJoin[{scriptDir, "phase2_stress_results.json"}]
];

numCases = 8;
sampleCount = 45;
interpGrid = N[Range[-6, 6, 0.5]];

thresholds = <|
  "pq_fit_max" -> 1.*^-5,
  "elim_residual_max" -> 1.*^-6,
  "char_det_max" -> 1.*^-8,
  "best_solution_residual_max" -> 1.*^-4
|>;

cases = Table[
  Module[
    {
      seed = 20260300 + i,
      ex, dh, qTrue, target, samples, x3True, pq, e45, m45, elimResidual, detResidual,
      res, sols, bestRes, pass
    },
    ex = GenerateRandomGeneralCase[seed];
    dh = ex["DH"];
    qTrue = ex["QTrue"];
    target = ForwardKinematics[dh, qTrue];
    samples = SampleAngles[sampleCount];
    x3True = Tan[qTrue[[3]]/2.0];
    pq = BuildPQAtX3[dh, target, x3True, samples];
    e45 = EliminateTheta12[pq["P"], pq["Q"]];
    m45 = Monomial45[qTrue[[4]], qTrue[[5]]];
    elimResidual = If[e45 === $Failed, Infinity, Max[Abs[e45.m45]]];
    detResidual = Abs[CharacteristicDetAtX3[dh, target, x3True, samples]];
    res = SolveIKRR1993[
      dh,
      target,
      RRSampleCount -> sampleCount,
      RRInterpolationGrid -> interpGrid,
      RRResidualTolerance -> 1.*^-4
    ];
    sols = res["Solutions"];
    bestRes = If[Length[sols] > 0, Min[sols[[All, "residual"]]], Infinity];
    pass =
      pq["MaxResidual"] < thresholds["pq_fit_max"] &&
      elimResidual < thresholds["elim_residual_max"] &&
      detResidual < thresholds["char_det_max"] &&
      Length[sols] >= 1 &&
      bestRes < thresholds["best_solution_residual_max"];

    <|
      "seed" -> seed,
      "pq_fit_residual" -> N[pq["MaxResidual"], 20],
      "elim_residual" -> N[elimResidual, 20],
      "char_det_residual" -> N[detResidual, 20],
      "num_real_x3_roots" -> Length[res["RealX3Roots"]],
      "num_solutions" -> Length[sols],
      "best_solution_residual" -> N[bestRes, 20],
      "pass" -> pass
    |>
  ],
  {i, 1, numCases}
];

(* Mandatory Manseur-Doty 16-real-solution reference case *)
dh16 = {
  {0.3, Pi/2, 0.0},
  {1.0, 1.0, 0.0},
  {0.0, Pi/2, 0.2},
  {1.5, 1.0, 0.0},
  {0.0, Pi/2, 0.0},
  {0.0, 1.0, 0.0}
};
q16 = {0.0, 107.458 Degree, 112.460 Degree, -7.662 Degree, 0.0, 0.0};
target16 = ForwardKinematics[dh16, q16];
res16 = SolveIKRR1993[
  dh16,
  target16,
  RRSampleCount -> 50,
  RRInterpolationGrid -> N[Range[-8, 8, 0.5]],
  RRResidualTolerance -> 1.*^-4
];
sols16 = res16["Solutions"];
bestRes16 = If[Length[sols16] > 0, Min[sols16[[All, "residual"]]], Infinity];
pass16 = Length[res16["RealX3Roots"]] >= 16 && Length[sols16] >= 15 && bestRes16 < 1.*^-8;

summary = <|
  "thresholds" -> thresholds,
  "num_random_cases" -> numCases,
  "num_random_passed" -> Count[cases[[All, "pass"]], True],
  "random_cases" -> cases,
  "case_16_real_solution" -> <|
    "num_real_x3_roots" -> Length[res16["RealX3Roots"]],
    "num_solutions" -> Length[sols16],
    "best_solution_residual" -> N[bestRes16, 20],
    "pass" -> pass16
  |>
|>;

Export[outPath, summary, "JSON"];
Print["Wrote Phase 2 stress JSON: ", outPath];
Print["Random passed: ", summary["num_random_passed"], "/", numCases];
Print[
  "16-root case: roots=",
  summary["case_16_real_solution", "num_real_x3_roots"],
  ", solutions=",
  summary["case_16_real_solution", "num_solutions"],
  ", pass=",
  summary["case_16_real_solution", "pass"]
];

overallPass = summary["num_random_passed"] == numCases && pass16;
If[overallPass, Exit[0], Exit[1]];

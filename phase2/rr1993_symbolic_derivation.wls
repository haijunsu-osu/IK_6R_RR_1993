(* ::Package:: *)
(* Raghavan-Roth 1993 resultant-based IK pipeline for a general 6R chain.
   This script keeps runtime numeric, but the elimination structure follows:
   p,l -> 14 equations -> eliminate {theta1,theta2} -> dialytic elimination in {x4,x5}
   -> characteristic polynomial in x3 = tan(theta3/2). *)

With[{rrNoAutoRunFlag = TrueQ[$RR1993NoAutoRun]},
  ClearAll["Global`*"];
  $RR1993NoAutoRun = rrNoAutoRunFlag;
];

(* --------------------------- Basic rigid transforms ----------------------- *)

NormalizeAngle[th_] := Module[{x = N[th]},
  Mod[x + Pi, 2 Pi] - Pi
];

NormalizePair[{s_, c_}] := Module[{n = Sqrt[s*s + c*c]},
  If[n < 10^-14, {0., 1.}, {s/n, c/n}]
];

SE3Inverse[T_] := Module[{r, p, tInv},
  r = T[[1 ;; 3, 1 ;; 3]];
  p = Flatten[T[[1 ;; 3, 4]]];
  tInv = IdentityMatrix[4];
  tInv[[1 ;; 3, 1 ;; 3]] = Transpose[r];
  tInv[[1 ;; 3, 4]] = -(Transpose[r].p);
  tInv
];

Aiv[th_] := {
  {Cos[th], -Sin[th], 0., 0.},
  {Sin[th],  Cos[th], 0., 0.},
  {0.,       0.,      1., 0.},
  {0.,       0.,      0., 1.}
};

Ais[a_, alpha_, d_] := {
  {1., 0.,          0.,         a},
  {0., Cos[alpha], -Sin[alpha], 0.},
  {0., Sin[alpha],  Cos[alpha], d},
  {0., 0.,          0.,         1.}
};

DHMatrix[a_, alpha_, d_, th_] := Aiv[th].Ais[a, alpha, d];

ForwardKinematics[dh_, q_] := Module[{a},
  a = Table[DHMatrix[dh[[i, 1]], dh[[i, 2]], dh[[i, 3]], q[[i]]], {i, 1, 6}];
  Fold[Dot, IdentityMatrix[4], a]
];

(* -------------------------- p/l equations (paper) ------------------------- *)

BuildPLVectors[dh_, target_, q_] := Module[
  {a, a2v, a2s, left, right, pL, pR, lL, lR},
  a = Table[DHMatrix[dh[[i, 1]], dh[[i, 2]], dh[[i, 3]], q[[i]]], {i, 1, 6}];
  a2v = Aiv[q[[2]]];
  a2s = Ais[dh[[2, 1]], dh[[2, 2]], dh[[2, 3]]];
  left = a2s.a[[3]].a[[4]].a[[5]];
  right = Inverse[a2v].Inverse[a[[1]]].target.Inverse[a[[6]]];
  pL = left[[1 ;; 3, 4]];
  pR = right[[1 ;; 3, 4]];
  lL = left[[1 ;; 3, 3]];
  lR = right[[1 ;; 3, 3]];
  {N[pL], N[pR], N[lL], N[lR]}
];

Build14Equations[dh_, target_, q_] := Module[
  {pL, pR, lL, lR, eqP, eqL, eqPP, eqPL, eqPX, comboL, comboR, eqCombo},
  {pL, pR, lL, lR} = BuildPLVectors[dh, target, q];
  eqP = pL - pR;
  eqL = lL - lR;
  eqPP = pL.pL - pR.pR;
  eqPL = pL.lL - pR.lR;
  eqPX = Cross[pL, lL] - Cross[pR, lR];
  comboL = (pL.pL)*lL - 2*(pL.lL)*pL;
  comboR = (pR.pR)*lR - 2*(pR.lR)*pR;
  eqCombo = comboL - comboR;
  Join[eqP, eqL, {eqPP, eqPL}, eqPX, eqCombo]
];

(* ------------------------------- Monomials -------------------------------- *)

Monomial45[th4_, th5_] := {
  Sin[th4]*Sin[th5],
  Sin[th4]*Cos[th5],
  Cos[th4]*Sin[th5],
  Cos[th4]*Cos[th5],
  Sin[th4],
  Cos[th4],
  Sin[th5],
  Cos[th5],
  1.
};

Monomial12[th1_, th2_] := {
  Sin[th1]*Sin[th2],
  Sin[th1]*Cos[th2],
  Cos[th1]*Sin[th2],
  Cos[th1]*Cos[th2],
  Sin[th1],
  Cos[th1],
  Sin[th2],
  Cos[th2]
};

(* x4/x5 polynomial basis after clearing ((1+x4^2)(1+x5^2)) *)
BuildHalfAngle45Map[] := Module[{t = ConstantArray[0., {9, 9}]},
  (* basis: {x4^2 x5^2, x4^2 x5, x4 x5^2, x4 x5, x4^2, x5^2, x4, x5, 1} *)
  (* s4 s5 *)
  t[[1, 4]] = 4.;
  (* s4 c5 *)
  t[[2, 3]] = -2.; t[[2, 7]] = 2.;
  (* c4 s5 *)
  t[[3, 2]] = -2.; t[[3, 8]] = 2.;
  (* c4 c5 *)
  t[[4, 1]] = 1.; t[[4, 5]] = -1.; t[[4, 6]] = -1.; t[[4, 9]] = 1.;
  (* s4 *)
  t[[5, 3]] = 2.; t[[5, 7]] = 2.;
  (* c4 *)
  t[[6, 1]] = -1.; t[[6, 5]] = -1.; t[[6, 6]] = 1.; t[[6, 9]] = 1.;
  (* s5 *)
  t[[7, 2]] = 2.; t[[7, 8]] = 2.;
  (* c5 *)
  t[[8, 1]] = -1.; t[[8, 5]] = 1.; t[[8, 6]] = -1.; t[[8, 9]] = 1.;
  (* 1 *)
  t[[9, 1]] = 1.; t[[9, 5]] = 1.; t[[9, 6]] = 1.; t[[9, 9]] = 1.;
  t
];

(* -------------------------- Coefficient extraction ------------------------ *)

SampleAngles[n_] := Module[{},
  SeedRandom[73421];
  Table[RandomReal[{-Pi, Pi}, 4], {n}]
];

EquationValuesAtX3[dh_, target_, x3_, th1_, th2_, th4_, th5_] := Module[{q},
  q = {th1, th2, 2 ArcTan[x3], th4, th5, 0.0};
  Build14Equations[dh, target, q]
];

BuildPQAtX3[dh_, target_, x3_, samples_] := Module[
  {features, values, pRows, qRows, residuals, fMat, rowVals, sol},
  fMat = N[
    Table[
      Join[
        Monomial45[s[[3]], s[[4]]],
        -Monomial12[s[[1]], s[[2]]]
      ],
      {s, samples}
    ],
    50
  ];
  values = N[
    Table[
      EquationValuesAtX3[dh, target, x3, s[[1]], s[[2]], s[[3]], s[[4]]],
      {s, samples}
    ],
    50
  ];
  pRows = ConstantArray[0., {14, 9}];
  qRows = ConstantArray[0., {14, 8}];
  residuals = ConstantArray[0., 14];
  Do[
    rowVals = values[[All, i]];
    sol = LeastSquares[fMat, rowVals];
    pRows[[i]] = sol[[1 ;; 9]];
    qRows[[i]] = sol[[10 ;; 17]];
    residuals[[i]] = Max[Abs[fMat.sol - rowVals]];
    ,
    {i, 1, 14}
  ];
  <|
    "P" -> pRows,
    "Q" -> qRows,
    "MaxResidual" -> Max[residuals],
    "RowResiduals" -> residuals
  |>
];

SelectIndependentRows[m_, k_] := Module[
  {idx = {}, remaining, bestRow, bestScore, cand, block, rank, sv, score},
  remaining = Range[Length[m]];
  While[Length[idx] < k && Length[remaining] > 0,
    bestRow = Missing["NotFound"];
    bestScore = Infinity;
    Do[
      cand = Append[idx, r];
      block = m[[cand]];
      rank = MatrixRank[block, Tolerance -> 10^-10];
      If[rank < Length[cand], Continue[]];
      sv = SingularValueList[N[block, 40]];
      If[Length[sv] == 0 || Abs[Last[sv]] <= 10^-14, Continue[]];
      score = First[sv]/Last[sv];
      If[score < bestScore,
        bestScore = score;
        bestRow = r;
      ];
      ,
      {r, remaining}
    ];
    If[bestRow === Missing["NotFound"], Break[]];
    idx = Append[idx, bestRow];
    remaining = DeleteCases[remaining, bestRow];
  ];
  idx
];

EliminateTheta12[p_, q_] := Module[
  {leftNull},
  leftNull = NullSpace[Transpose[q], Tolerance -> 10^-12];
  If[Length[leftNull] < 6,
    Return[$Failed]
  ];
  leftNull.p
];

BuildLeftEliminationMatrix[q_, pivotRows_] := Module[
  {piv, rem, qP, qR, x, nPerm, order, n},
  piv = pivotRows;
  If[Length[piv] != 8 || Length[DeleteDuplicates[piv]] != 8, Return[$Failed]];
  If[Min[piv] < 1 || Max[piv] > Length[q], Return[$Failed]];
  rem = Complement[Range[Length[q]], piv];
  qP = q[[piv]];
  If[MatrixRank[qP, Tolerance -> 10^-10] < 8, Return[$Failed]];
  qR = q[[rem]];
  x = Quiet[Check[Transpose[LinearSolve[Transpose[qP], Transpose[qR]]], $Failed]];
  If[x === $Failed, Return[$Failed]];
  nPerm = ArrayFlatten[{{-x, IdentityMatrix[Length[rem]]}}];
  order = Join[piv, rem];
  n = ConstantArray[0., {Length[rem], Length[q]}];
  Do[n[[All, order[[j]]]] = nPerm[[All, j]], {j, 1, Length[order]}];
  n
];

BuildDialytic12x12[e9_] := Module[{m = ConstantArray[0., {12, 12}], c, r},
  Do[
    c = e9[[i]];
    r = ConstantArray[0., 12];
    (* non-multiplied equation *)
    r[[4]] = c[[1]];
    r[[5]] = c[[2]];
    r[[7]] = c[[3]];
    r[[8]] = c[[4]];
    r[[6]] = c[[5]];
    r[[10]] = c[[6]];
    r[[9]] = c[[7]];
    r[[11]] = c[[8]];
    r[[12]] = c[[9]];
    m[[i]] = r;
    (* multiplied by x4 *)
    r = ConstantArray[0., 12];
    r[[1]] = c[[1]];
    r[[2]] = c[[2]];
    r[[4]] = c[[3]];
    r[[5]] = c[[4]];
    r[[3]] = c[[5]];
    r[[7]] = c[[6]];
    r[[6]] = c[[7]];
    r[[8]] = c[[8]];
    r[[9]] = c[[9]];
    m[[i + 6]] = r;
    ,
    {i, 1, 6}
  ];
  m
];

BuildPQSinCosModel[dh_, target_, samples_] := Module[
  {
    theta3Grid, basis, basisInv, pSamples = {}, qSamples = {},
    x3, pq, pS, pC, p1, qConst, qSpread, pFitErr = 0., pRec, i
  },
  theta3Grid = N[{-2.1, -0.2, 1.3}];
  basis = Transpose[{Sin[theta3Grid], Cos[theta3Grid], ConstantArray[1., 3]}];
  basisInv = Inverse[basis];
  Do[
    x3 = Tan[theta3/2.0];
    pq = BuildPQAtX3[dh, target, x3, samples];
    AppendTo[pSamples, pq["P"]];
    AppendTo[qSamples, pq["Q"]];
    ,
    {theta3, theta3Grid}
  ];
  pS =
    basisInv[[1, 1]]*pSamples[[1]] +
    basisInv[[1, 2]]*pSamples[[2]] +
    basisInv[[1, 3]]*pSamples[[3]];
  pC =
    basisInv[[2, 1]]*pSamples[[1]] +
    basisInv[[2, 2]]*pSamples[[2]] +
    basisInv[[2, 3]]*pSamples[[3]];
  p1 =
    basisInv[[3, 1]]*pSamples[[1]] +
    basisInv[[3, 2]]*pSamples[[2]] +
    basisInv[[3, 3]]*pSamples[[3]];
  qConst = Mean[qSamples];
  qSpread = Max[Abs[Flatten[Table[qSamples[[i]] - qConst, {i, 1, Length[qSamples]}]]]];
  Do[
    pRec = pS*Sin[theta3Grid[[i]]] + pC*Cos[theta3Grid[[i]]] + p1;
    pFitErr = Max[pFitErr, Max[Abs[pRec - pSamples[[i]]]]];
    ,
    {i, 1, Length[theta3Grid]}
  ];
  <|
    "PS" -> pS,
    "PC" -> pC,
    "P1" -> p1,
    "QConst" -> qConst,
    "QTheta3Spread" -> qSpread,
    "PSinCosFitError" -> pFitErr
  |>
];

BuildMatrixPolynomialPencilQuadratic[dh_, target_, samples_, xSamples_] := Module[
  {
    model, pS, pC, p1, qConst, pivotRows, nMat, map45,
    p2, p1lin, p0, e92, e91, e90, m2, m1, m0,
    maxErr = 0., x, pq, e45, e9, mDirect, lhs, rhs
  },
  model = BuildPQSinCosModel[dh, target, samples];
  pS = model["PS"];
  pC = model["PC"];
  p1 = model["P1"];
  qConst = model["QConst"];
  pivotRows = SelectIndependentRows[qConst, 8];
  If[Length[pivotRows] != 8, Return[$Failed]];
  nMat = BuildLeftEliminationMatrix[qConst, pivotRows];
  If[nMat === $Failed, Return[$Failed]];
  map45 = BuildHalfAngle45Map[];
  p2 = p1 - pC;
  p1lin = 2.0*pS;
  p0 = p1 + pC;
  e92 = nMat.p2.map45;
  e91 = nMat.p1lin.map45;
  e90 = nMat.p0.map45;
  m2 = BuildDialytic12x12[e92];
  m1 = BuildDialytic12x12[e91];
  m0 = BuildDialytic12x12[e90];
  Do[
    pq = BuildPQAtX3[dh, target, x, samples];
    e45 = nMat.pq["P"];
    e9 = e45.map45;
    mDirect = BuildDialytic12x12[e9];
    lhs = (1.0 + x^2)*mDirect;
    rhs = m0 + m1*x + m2*x^2;
    maxErr = Max[maxErr, Max[Abs[lhs - rhs]]];
    ,
    {x, xSamples}
  ];
  <|
    "M0" -> m0,
    "M1" -> m1,
    "M2" -> m2,
    "PivotRows" -> pivotRows,
    "FitMaxEntryError" -> maxErr,
    "QTheta3Spread" -> model["QTheta3Spread"],
    "PSinCosFitError" -> model["PSinCosFitError"]
  |>
];

DedupeScalarRoots[vals_, tol_: 10^-6] := Module[{sorted, out = {}},
  sorted = Sort[Re[Select[vals, NumericQ[#] && Abs[Im[#]] < 10^-12 &]]];
  Do[
    If[Length[out] == 0 || Abs[x - Last[out]] > tol, AppendTo[out, x]];
    ,
    {x, sorted}
  ];
  out
];

RefineRootNewton[detFunc_, x0_, maxIter_: 20] := Module[
  {x = N[x0], fx, h, fp, xNext},
  Do[
    fx = detFunc[x];
    If[!NumericQ[fx], Break[]];
    h = 10^-4*(1 + Abs[x]);
    fp = (detFunc[x + h] - detFunc[x - h])/(2*h);
    If[!NumericQ[fp] || Abs[fp] <= 10^-12, Break[]];
    xNext = x - fx/fp;
    If[!NumericQ[xNext], Break[]];
    If[Abs[xNext - x] <= 10^-10*(1 + Abs[x]),
      x = xNext;
      Break[];
    ];
    x = xNext;
    ,
    {maxIter}
  ];
  x
];

SolveX3RootsByMatrixPencil[dh_, target_, samples_, xSamples_] := Module[
  {
    data, m0, m1, m2, n, a, b, eigs, xBound, rawRoots,
    detFunc, refined, scored, nearZero, realRoots
  },
  data = BuildMatrixPolynomialPencilQuadratic[dh, target, samples, xSamples];
  If[data === $Failed, Return[$Failed]];
  m0 = data["M0"];
  m1 = data["M1"];
  m2 = data["M2"];
  n = Length[m0];
  a = ConstantArray[0., {2*n, 2*n}];
  b = ConstantArray[0., {2*n, 2*n}];
  a[[1 ;; n, 1 ;; n]] = -m1;
  a[[1 ;; n, n + 1 ;; 2*n]] = -m0;
  b[[1 ;; n, 1 ;; n]] = m2;
  a[[n + 1 ;; 2*n, 1 ;; n]] = IdentityMatrix[n];
  b[[n + 1 ;; 2*n, n + 1 ;; 2*n]] = IdentityMatrix[n];
  eigs = Quiet[N[Eigenvalues[{a, b}], 40]];
  xBound = 4.0*Max[1.0, Max[Abs[N[xSamples]]]];
  rawRoots = Re[
    Select[
      eigs,
      NumericQ[#] && Abs[Im[#]] <= 10^-7 && Abs[Re[#]] <= xBound &
    ]
  ];
  rawRoots = DedupeScalarRoots[rawRoots, 10^-4];
  detFunc[x_] := CharacteristicDetAtX3[dh, target, x, samples];
  refined = DedupeScalarRoots[RefineRootNewton[detFunc, #] & /@ rawRoots, 10^-5];
  scored = SortBy[
    Table[{Abs[detFunc[x]], x}, {x, refined}],
    First
  ];
  nearZero = Cases[scored, {err_, x_} /; NumericQ[err] && err <= 10^-6 :> x];
  realRoots = If[Length[nearZero] > 0, nearZero, If[Length[scored] > 0, scored[[All, 2]], {}]];
  realRoots = DedupeScalarRoots[realRoots, 10^-5];
  <|
    "RealRoots" -> realRoots,
    "PivotRows" -> data["PivotRows"],
    "Diagnostics" -> data
  |>
];

CharacteristicDetAtX3[dh_, target_, x3_, samples_] := Module[
  {pq, e45, map45, e9, dMat, d},
  pq = BuildPQAtX3[dh, target, x3, samples];
  e45 = EliminateTheta12[pq["P"], pq["Q"]];
  If[e45 === $Failed, Return[Indeterminate]];
  map45 = BuildHalfAngle45Map[];
  e9 = e45.map45;
  dMat = BuildDialytic12x12[e9];
  d = Quiet[N[Det[dMat], 50], Det::luc];
  d
];

E9Polynomial[row_, x4_, x5_] :=
  row[[1]]*x4^2*x5^2 + row[[2]]*x4^2*x5 + row[[3]]*x4*x5^2 + row[[4]]*x4*x5 +
  row[[5]]*x4^2 + row[[6]]*x5^2 + row[[7]]*x4 + row[[8]]*x5 + row[[9]];

SolveX4X5FromE9[e9_] := Module[
  {x4, x5, cands = {}, eq1, eq2, res, roots4, a, b, c, disc, roots5, vec, rErr, nr, eqList, rules},
  Do[
    eq1 = E9Polynomial[e9[[i]], x4, x5];
    eq2 = E9Polynomial[e9[[j]], x4, x5];
    res = Expand[Resultant[eq1, eq2, x5]];
    If[res === 0, Continue[]];
    nr = NRoots[res == 0, x4];
    eqList = If[Head[nr] === Or, List @@ nr, nr];
    rules = eqList /. Equal -> Rule;
    roots4 = Flatten@{x4 /. rules};
    roots4 = Select[roots4, NumericQ[#] && Abs[Im[#]] < 10^-7 &] // Re;
    Do[
      a = N[Coefficient[eq1, x5, 2] /. x4 -> r4];
      b = N[Coefficient[eq1, x5, 1] /. x4 -> r4];
      c = N[Coefficient[eq1, x5, 0] /. x4 -> r4];
      roots5 = {};
      If[Abs[a] < 10^-10,
        If[Abs[b] > 10^-10, roots5 = {-c/b};];
        ,
        disc = b^2 - 4*a*c;
        If[Re[disc] >= -10^-10,
          disc = If[Re[disc] < 0, 0, Re[disc]];
          roots5 = {(-b + Sqrt[disc])/(2*a), (-b - Sqrt[disc])/(2*a)};
        ];
      ];
      roots5 = Select[roots5, NumericQ[#] && Abs[Im[#]] < 10^-7 &] // Re;
      Do[
        vec = {r4^2*r5^2, r4^2*r5, r4*r5^2, r4*r5, r4^2, r5^2, r4, r5, 1.0};
        rErr = Max[Abs[e9.vec]];
        If[rErr < 10^-3, AppendTo[cands, {r4, r5}]];
        ,
        {r5, roots5}
      ];
      ,
      {r4, roots4}
    ];
    ,
    {i, 1, 5}, {j, i + 1, 6}
  ];
  DeleteDuplicates[cands, (Abs[#1[[1]] - #2[[1]]] < 10^-6 && Abs[#1[[2]] - #2[[2]]] < 10^-6 &)]
];

CharacteristicPolynomialByInterpolation[dh_, target_, samples_, xSamples_] := Module[
  {vals, x, vander, coeffAsc, poly},
  vals = Table[CharacteristicDetAtX3[dh, target, xv, samples], {xv, xSamples}];
  vander = N[
    Table[If[k == 0, 1.0, xSamples[[i]]^k], {i, 1, Length[xSamples]}, {k, 0, 16}],
    50
  ];
  coeffAsc = LeastSquares[vander, N[vals, 50]];
  poly = Sum[coeffAsc[[k + 1]]*x^k, {k, 0, 16}] // Expand;
  <|"Polynomial" -> poly, "CoefficientsAscending" -> coeffAsc, "SampleValues" -> vals|>
];

BracketDeterminantRoots[dh_, target_, samples_, xmin_, xmax_, nPts_: 121] := Module[
  {xs, fs, roots = {}, a, b, fa, fb, left, right, fLeft, fMid, mid},
  xs = N[Subdivide[xmin, xmax, nPts - 1]];
  fs = Table[CharacteristicDetAtX3[dh, target, x, samples], {x, xs}];
  Do[
    a = xs[[i]];
    b = xs[[i + 1]];
    fa = fs[[i]];
    fb = fs[[i + 1]];
    If[!NumericQ[fa] || !NumericQ[fb], Continue[]];
    If[Abs[fa] < 10^-10, AppendTo[roots, a]; Continue[]];
    If[fa*fb > 0, Continue[]];
    left = a;
    right = b;
    fLeft = fa;
    Do[
      mid = (left + right)/2;
      fMid = CharacteristicDetAtX3[dh, target, mid, samples];
      If[!NumericQ[fMid], Break[]];
      If[Abs[fMid] < 10^-12, left = right = mid; Break[]];
      If[fLeft*fMid <= 0,
        right = mid;
        ,
        left = mid;
        fLeft = fMid;
      ];
      ,
      {60}
    ];
    AppendTo[roots, (left + right)/2];
    ,
    {i, 1, Length[xs] - 1}
  ];
  DeleteDuplicates[roots, (Abs[#1 - #2] < 10^-5 &)]
];

SolveX4X5FromNullVector[z_] := Module[{cand = {}, zc = N[z]},
  If[Abs[zc[[12]]] > 10^-10, AppendTo[cand, {zc[[9]]/zc[[12]], zc[[11]]/zc[[12]]}]];
  If[Abs[zc[[9]]] > 10^-10, AppendTo[cand, {zc[[6]]/zc[[9]], zc[[8]]/zc[[9]]}]];
  If[Abs[zc[[11]]] > 10^-10, AppendTo[cand, {zc[[8]]/zc[[11]], zc[[10]]/zc[[11]]}]];
  Select[
    DeleteDuplicatesBy[cand, (Round[#, 10^-8] &)],
    VectorQ[#, NumericQ] && Max[Abs[#]] < 10^8 &
  ]
];

Monomial12FromX45[x4_, x5_] := {
  x4^3*x5^2,
  x4^3*x5,
  x4^3,
  x4^2*x5^2,
  x4^2*x5,
  x4^2,
  x4*x5^2,
  x4*x5,
  x4,
  x5^2,
  x5,
  1.0
};

Monomial9FromX45[x4_, x5_] := {
  x4^2*x5^2,
  x4^2*x5,
  x4*x5^2,
  x4*x5,
  x4^2,
  x5^2,
  x4,
  x5,
  1.0
};

SolveX4X5FromEq14Eigenvectors[dMat_, e9_, maxVectors_: 8] := Module[
  {vals, vecs, order, cands = {}, candFromVec, x4, x5, z12, z9, err14, err9, ns},
  {vals, vecs} = Quiet[Check[Eigensystem[N[dMat, 40]], {{$Failed}, {{$Failed}}}]];
  If[vals =!= {$Failed},
    order = Ordering[Abs[vals], UpTo[maxVectors]];
    Do[
      If[Max[Abs[Im[vecs[[k]]]]] > 10^-8, Continue[]];
      candFromVec = SolveX4X5FromNullVector[Re[vecs[[k]]]];
      Do[
        x4 = xcand[[1]];
        x5 = xcand[[2]];
        z12 = Monomial12FromX45[x4, x5];
        z9 = Monomial9FromX45[x4, x5];
        err14 = Max[Abs[dMat.z12]];
        err9 = Max[Abs[e9.z9]];
        If[err14 <= 10^-5 && err9 <= 10^-3, AppendTo[cands, {x4, x5}]];
        ,
        {xcand, candFromVec}
      ];
      ,
      {k, order}
    ];
  ];
  If[Length[cands] == 0,
    ns = NullSpace[N[dMat, 40], Tolerance -> 10^-16];
    Do[
      candFromVec = SolveX4X5FromNullVector[z];
      Do[
        x4 = xcand[[1]];
        x5 = xcand[[2]];
        z12 = Monomial12FromX45[x4, x5];
        z9 = Monomial9FromX45[x4, x5];
        err14 = Max[Abs[dMat.z12]];
        err9 = Max[Abs[e9.z9]];
        If[err14 <= 10^-5 && err9 <= 10^-3, AppendTo[cands, {x4, x5}]];
        ,
        {xcand, candFromVec}
      ];
      ,
      {z, ns}
    ];
  ];
  DeleteDuplicates[cands, (Abs[#1[[1]] - #2[[1]]] < 10^-7 && Abs[#1[[2]] - #2[[2]]] < 10^-7 &)]
];

RecoverTheta12[pRows_, qRows_, th4_, th5_] := Module[
  {m45, rhs, n, sc1, sc2, th1, th2},
  m45 = Monomial45[th4, th5];
  rhs = pRows.m45;
  n = LeastSquares[qRows, rhs];
  sc1 = NormalizePair[{n[[5]], n[[6]]}];
  sc2 = NormalizePair[{n[[7]], n[[8]]}];
  th1 = NormalizeAngle[ArcTan[sc1[[2]], sc1[[1]]]];
  th2 = NormalizeAngle[ArcTan[sc2[[2]], sc2[[1]]]];
  {th1, th2}
];

RecoverTheta6[dh_, target_, q1_, q2_, q3_, q4_, q5_] := Module[
  {a1to5, a6target, c6, s6, qpre},
  qpre = {q1, q2, q3, q4, q5};
  a1to5 = Fold[
    Dot,
    IdentityMatrix[4],
    Table[
      DHMatrix[dh[[i, 1]], dh[[i, 2]], dh[[i, 3]], qpre[[i]]],
      {i, 1, 5}
    ]
  ];
  a6target = SE3Inverse[a1to5].target;
  c6 = a6target[[1, 1]];
  s6 = a6target[[2, 1]];
  NormalizeAngle[ArcTan[c6, s6]]
];

PoseResidual[dh_, target_, q_] := Module[{fk},
  fk = ForwardKinematics[dh, q];
  Norm[Flatten[fk - target]]
];

SolveIKRR1993[dh_, target_, opts : OptionsPattern[]] := Module[
  {
    sampleCount, xGrid, samples, charData, roots, realRoots = {}, sols = {},
    bracketRoots, x3RootData, x3RootMethod = "matrix_pencil",
    th3, pq, e45, map45, e9, dMat, ns, x45, nullCands, th4, th5, th12, th1, th2, th6,
    q, res, tol, unique, var
  },
  sampleCount = OptionValue[RRSampleCount];
  xGrid = OptionValue[RRInterpolationGrid];
  tol = OptionValue[RRResidualTolerance];
  samples = SampleAngles[sampleCount];
  charData = CharacteristicPolynomialByInterpolation[dh, target, samples, xGrid];
  x3RootData = SolveX3RootsByMatrixPencil[dh, target, samples, xGrid];
  If[x3RootData === $Failed || Length[x3RootData["RealRoots"]] == 0,
    x3RootMethod = "interpolation_bracket_fallback";
    var = First[Variables[charData["Polynomial"]]];
    roots = Flatten@{Module[{nr, eqList, rules},
      nr = NRoots[charData["Polynomial"] == 0, var];
      eqList = If[Head[nr] === Or, List @@ nr, nr];
      rules = eqList /. Equal -> Rule;
      var /. rules
    ]};
    realRoots = Select[roots, NumericQ[#] && Abs[Im[#]] < 10^-7 &] // Re;
    bracketRoots = BracketDeterminantRoots[dh, target, samples, Min[xGrid], Max[xGrid], 241];
    realRoots = Join[realRoots, bracketRoots];
    ,
    realRoots = x3RootData["RealRoots"];
  ];
  realRoots = DedupeScalarRoots[realRoots, 10^-5];

  Do[
    th3 = NormalizeAngle[2 ArcTan[x3]];
    pq = BuildPQAtX3[dh, target, x3, samples];
    e45 = EliminateTheta12[pq["P"], pq["Q"]];
    If[e45 === $Failed, Continue[]];
    map45 = BuildHalfAngle45Map[];
    e9 = e45.map45;
    dMat = BuildDialytic12x12[e9];
    x45 = SolveX4X5FromEq14Eigenvectors[dMat, e9, 8];
    If[Length[x45] == 0, x45 = SolveX4X5FromE9[e9]];
    If[Length[x45] == 0,
      ns = NullSpace[N[dMat, 40], Tolerance -> 10^-16];
      If[Length[ns] == 0, Continue[]];
      nullCands = Flatten[Table[SolveX4X5FromNullVector[z], {z, ns}], 1];
      x45 = DeleteDuplicates[
        nullCands,
        (Abs[#1[[1]] - #2[[1]]] < 10^-6 && Abs[#1[[2]] - #2[[2]]] < 10^-6 &)
      ];
      If[Length[x45] == 0, Continue[]];
    ];
    Do[
      th4 = NormalizeAngle[2 ArcTan[xcand[[1]]]];
      th5 = NormalizeAngle[2 ArcTan[xcand[[2]]]];
      th12 = RecoverTheta12[pq["P"], pq["Q"], th4, th5];
      th1 = th12[[1]];
      th2 = th12[[2]];
      th6 = RecoverTheta6[dh, target, th1, th2, th3, th4, th5];
      q = {th1, th2, th3, th4, th5, th6};
      res = PoseResidual[dh, target, q];
      If[res < tol, AppendTo[sols, <|"q" -> q, "residual" -> res|>]];
      ,
      {xcand, x45}
    ];
    ,
    {x3, realRoots}
  ];

  unique = DeleteDuplicatesBy[
    sols,
    (Round[NormalizeAngle /@ #["q"], 10^-6] &)
  ];
  <|
    "Solutions" -> unique,
    "CharacteristicPolynomial" -> charData["Polynomial"],
    "RealX3Roots" -> realRoots,
    "X3RootMethod" -> x3RootMethod
  |>
];

Options[SolveIKRR1993] = {
  RRSampleCount -> 60,
  RRInterpolationGrid -> N[Range[-6, 6, 0.5]],
  RRResidualTolerance -> 10^-5
};

(* ------------------------------- CLI demo -------------------------------- *)

If[$FrontEnd === Null && !TrueQ[$RR1993NoAutoRun],
  Print["rr1993_combined_examples.wls loaded."];
  Print["Use SolveIKRR1993[dh, target] with dh = {{a1,alpha1,d1},...,{a6,alpha6,d6}}."]
];

(* ------------------------ Combined verification demo ---------------------- *)

AngleDiff[a_, b_] := NormalizeAngle[a - b];

PoseDistance[q1_, q2_] := Norm[Table[AngleDiff[q1[[i]], q2[[i]]], {i, 1, 6}]];

GenerateRandomGeneralCase[seed_Integer : 20260225] := BlockRandom[
  SeedRandom[seed];
  <|
    "DH" -> Transpose[{
      RandomReal[{0.08, 0.45}, 6],
      RandomChoice[{-1., 1.}, 6]*RandomReal[{0.35, 1.35}, 6],
      RandomReal[{0.0, 0.30}, 6]
    }],
    "QTrue" -> RandomReal[{-Pi, Pi}, 6]
  |>
];

RunCase[
  caseName_,
  dhCase_,
  qTrueCase_,
  sampleCount_Integer,
  interpGrid_,
  expectedTarget_ : Missing["NotProvided"],
  expectedMinRoots_Integer : 0
] := Module[
  {
    targetCase,
    samplesCase,
    x3TrueCase,
    pqCase,
    e45Case,
    m45TrueCase,
    elimResidualCase,
    detResidualCase,
    targetDiff = Indeterminate,
    resCase,
    solutionsCase,
    sortedSolutionsCase,
    distancesCase,
    bestIdxCase,
    bestCase,
    bestDistCase,
    passDerivation,
    passTarget,
    meetsExpectedRoots,
    passCase
  },
  Print[""];
  Print["=== Case: ", caseName, " ==="];

  targetCase = ForwardKinematics[dhCase, qTrueCase];
  If[MatrixQ[expectedTarget] && Dimensions[expectedTarget] == {4, 4},
    targetDiff = Max[Abs[targetCase - expectedTarget]];
    Print["Target matrix max abs diff vs published: ", N[targetDiff, 8]];
  ];

  samplesCase = SampleAngles[sampleCount];
  x3TrueCase = Tan[qTrueCase[[3]]/2.0];
  pqCase = BuildPQAtX3[dhCase, targetCase, x3TrueCase, samplesCase];
  e45Case = EliminateTheta12[pqCase["P"], pqCase["Q"]];
  m45TrueCase = Monomial45[qTrueCase[[4]], qTrueCase[[5]]];
  elimResidualCase = If[e45Case === $Failed, Infinity, Max[Abs[e45Case.m45TrueCase]]];
  detResidualCase = Abs[CharacteristicDetAtX3[dhCase, targetCase, x3TrueCase, samplesCase]];

  Print["Decomposition fit residual: ", N[pqCase["MaxResidual"], 8]];
  Print["Elimination residual at true (theta4,theta5): ", N[elimResidualCase, 8]];
  Print["Characteristic determinant at true x3: ", N[detResidualCase, 8]];

  resCase = SolveIKRR1993[
    dhCase,
    targetCase,
    RRSampleCount -> sampleCount,
    RRInterpolationGrid -> interpGrid,
    RRResidualTolerance -> 10^-4
  ];
  solutionsCase = resCase["Solutions"];
  sortedSolutionsCase = SortBy[solutionsCase, #["residual"] &];

  Print["Real x3 roots (solver): ", Length[resCase["RealX3Roots"]]];
  Print["Recovered IK solutions (solver): ", Length[sortedSolutionsCase]];
  If[expectedMinRoots > 0,
    meetsExpectedRoots = Length[resCase["RealX3Roots"]] >= expectedMinRoots;
    Print[
      "Expected real x3 roots >= ",
      expectedMinRoots,
      " ; observed = ",
      Length[resCase["RealX3Roots"]],
      " ; meets expectation: ",
      meetsExpectedRoots
    ];
    ,
    meetsExpectedRoots = Missing["NotChecked"];
  ];

  If[Length[sortedSolutionsCase] > 0,
    Print["All recovered IK solutions (q, residual):"];
    Do[
      Print[
        "  #",
        i,
        " q=",
        N[sortedSolutionsCase[[i, "q"]], 10],
        " residual=",
        N[sortedSolutionsCase[[i, "residual"]], 12]
      ];
      ,
      {i, 1, Length[sortedSolutionsCase]}
    ];
    distancesCase = Table[PoseDistance[qTrueCase, s["q"]], {s, sortedSolutionsCase}];
    bestIdxCase = First@Ordering[distancesCase, 1];
    bestCase = sortedSolutionsCase[[bestIdxCase]];
    bestDistCase = distancesCase[[bestIdxCase]];
    Print["Best angular distance to ground truth: ", N[bestDistCase, 8]];
    Print["Best solution residual: ", N[bestCase["residual"], 8]];
  ];

  passDerivation = elimResidualCase < 10^-6 && detResidualCase < 10^-8;
  passTarget = If[NumberQ[targetDiff], targetDiff < 5*10^-4, True];
  passCase = passDerivation && passTarget;

  Print["Case pass (derivation checks): ", passCase];

  <|
    "Pass" -> passCase,
    "MeetsExpectedRoots" -> meetsExpectedRoots,
    "RealX3Roots" -> Length[resCase["RealX3Roots"]],
    "NumSolutions" -> Length[sortedSolutionsCase]
  |>
];

RunCombinedExamples[] := Module[
  {
    randomExample,
    caseRandom,
    dh16,
    q16,
    target16Published,
    case16,
    overallPass
  },
  Print["Running RR1993 IK combined examples..."];

  randomExample = GenerateRandomGeneralCase[20260225];
  Print[""];
  Print["Random example seed: 20260225"];
  Print["Random DH:"];
  Print[MatrixForm[randomExample["DH"]]];
  Print["Random q_true:"];
  Print[N[randomExample["QTrue"], 8]];

  caseRandom = RunCase[
    "Seeded random general 6R example",
    randomExample["DH"],
    randomExample["QTrue"],
    45,
    N[Range[-6, 6, 0.5]]
  ];

  dh16 = {
    {0.3, Pi/2, 0.0},
    {1.0, 1.0, 0.0},
    {0.0, Pi/2, 0.2},
    {1.5, 1.0, 0.0},
    {0.0, Pi/2, 0.0},
    {0.0, 1.0, 0.0}
  };
  q16 = {0.0, 107.458 Degree, 112.460 Degree, -7.662 Degree, 0.0, 0.0};
  target16Published = {
    {-0.4655, -0.4822, 0.7421, -0.5377},
    {-0.6987, 0.7150, 0.0263, -1.1561},
    {-0.5433, -0.5063, -0.6697, 0.1895},
    {0.0, 0.0, 0.0, 1.0}
  };

  case16 = RunCase[
    "Manseur-Doty published 16-real-root example",
    dh16,
    q16,
    50,
    N[Range[-8, 8, 0.5]],
    target16Published,
    16
  ];

  Print[""];
  Print["--- Summary ---"];
  Print["Random case: roots=", caseRandom["RealX3Roots"], ", solutions=", caseRandom["NumSolutions"]];
  Print["16-root case: roots=", case16["RealX3Roots"], ", solutions=", case16["NumSolutions"], " (published reference has 16 real IK roots)"];

  overallPass = caseRandom["Pass"] && case16["Pass"];
  If[overallPass,
    Print["PASS: combined script checks succeeded for both examples."];
    ,
    Print["FAIL: combined script checks failed."];
  ];

  <|"OverallPass" -> overallPass, "RandomCase" -> caseRandom, "Case16" -> case16|>
];

If[$FrontEnd === Null && !TrueQ[$RR1993NoAutoRun],
  combinedSummary = RunCombinedExamples[];
  If[combinedSummary["OverallPass"], Exit[0], Exit[1]];
];

# Extracted Text: MCtra94.pdf

## Page 1

- extraction_method: `ocr_sidecar`
- char_count: `5532`

```text
89
IEEE TRANSACTIONS ON ROBOTICS AND AUTOMATION,VOL.10, NO.5, OCTOBER 1994
Efficient Inverse Kinematics
for General 6R Manipulators
Dinesh Manocha and John F.Canny
Abstract-The inverse kinematics of serial manipulators is athey suffer from two drawbacks.Firstly，they are slow for
central problem in the automatic control of robot manipulators.
practical applications, and secondly they are unable to find
Themain interesthasbeenininversekinematics of a sixrevolute
(6R） jointed manipulator with arbitrary geometry.It has been
all the solutions. As a result, most industrial manipulators are
recentlyshown thatthejointsofageneral6Rmanipulatorcan
designed sufficiently simply so that a closed form solution
orient themselves in 16 different configurations (at most),for
exists.
a given pose of the end-effector.However,there are no good
In the absence of a closed form solution,[26] claim that the
practical solutions available that give a level of performance
problem of inversekinematicsfor ageneral6R manipulator
expected of industrial manipulators. In this paper, we present
is considered solved under the following conditions.
analgorithmandimplementationforefficientinversekinematics
for a general 6Rmanipulator.When stated mathematically,the
1)Atightupperboundonthenumber of solutionshas
problem reduces to solving a system of multivariate equations.
been established.
Wemakeuse of the algebraic properties of the system and the
2) An efficient, numerically sound method for computing
symbolicformulation usedforreducingtheproblem to solvinga
univariate polynomial.However,the polynomial is expressed as a
all solutions has been developed.
matrix determinant and its roots are computed by reducing to an
At the same time,we feel it is important that the solution be
eigenvalueproblem.The other roots of the multivariate system
able toprovide a levelof performance expectedofindustrial
are obtained by computing eigenvectors and substitution.The
manipulators.
algorithm involves symbolic preprocessing,matrix computations
anda variety ofothernumerical techniques.The average running
The need for fast algorithms for inverse kinematics of
time of the algorithm, for most cases, is 11 milliseconds on an
general manipulators has been felt in kinematic design,kine-
IBM RS/6000 workstation.This approach is applicable to inverse
matic calibration andgoal-directedcomputeranimation as
kinematics of all serial manipulators.
configuration of manipulators given a set of kinematic task
1. INTRODUCTION
specifications [16], [17]. Given the kinematic requirements as
workspace volume, maximum reach and maximum positional
NHEINVERSE kinematics problemfor general serial ma-
【nipulatorsisfundamentalfor computer controlled robots.
tions,whose variables are the manipulator parameters [17].
Given the pose of the end effector (the position and ori- The current solutions are restricted to 6R manipulators with
entation)，the problem corresponds to computing the joint
closed form solutions,which limits the class of manipulators
displacements for that pose. The most interesting case has been
that can be used for kinematic design [17].
thatof serial manipulators withsixjoints.The complexity of
Theneedforkinematic calibration arises due tomanufac-
inverse kinematics of a general six jointed manipulator is a
turing errors in machining and assembly of manipulators. This
function of its geometry.While the solution can be expressed
results in discrepancies between the design parameters and
in closed form for a variety of special cases,such as when
physicalstructureandcanproducesignificanterrorsbetween
three consecutive axes intersect in a common point, no such
the actual and predicted positions and orientations of the end
formulation is known for the general case. The main interest
effector.The solution to this problem involves identification
hasbeenin a 6Rmanipulator,thathas sixrevolute joints,the
of theindividual kinematic parameters and incorporating them
links are of arbitrary length and no constraints are imposed
on the geometry of various links. Iterative solutions (based on
Given the accuratekinematic parameters,a numberof methods
numerical techniques) to the inversekinematicsforgeneral6R
have been proposed to calibrate and compensate for the
manipulators have been known for quite some time. However, kinematic errors in robot manipulators with closed form solu-
Manuscript received April 13,1993;revised October 12, 1993. This work
tions [8], [25]. However, a practical solution for the inverse
was supportedinpart by anIBMGraduateFellowship,underONRContract
kinematics of general manipulators eliminates the need for any
N00014-941-0738,a NSF Grant CCR-9319957,ARPA Contract DABT63-
algorithms for compensation of kinematic errors.
93-C-0048, and a grant from Mitsubishi Electric Research Lab, as well
as a David and Lucile Packard Fellowship, and a NSF Presidential Young
The inverse kinematics problem for six revolute joints has
Investigator Award (# IRI-8958577).
beenstudiedformorethantwodecades.The carlierwork
D.Manochaiswith theDepartment of ComputerScience at theUniversity
includes that of Pieper [18] and Roth et al.[22]. The first
of North Carolina, Chapel Hill, NC 27599-3175 USA.
J.Canny is withthe ComputerScience Division,University of California,
constructive solution to the problem was givenby[1],in the
Berkeley CA 94720 USA.
formof determinant of a12×12matrix,whose entries were
IEEE Log Number 9403325.
quarticpolynomialsinthe tangentof thehalf-angleof one of
1042296X/94$04.00 ? 1994 IEEE
```

## Page 2

- extraction_method: `ocr_sidecar`
- char_count: `5553`

```text
MANOCHA AND CANNY: EFFICIENT INVERSE KINEMATICS FOR GENERAL 6R MANIPULATORS
649
the joint variables. Later [5] provided a 32 degree polynomial
The rest of the paper is organized in the following manner.
in the tangent of the half-angle of one of the joint variables.In Section II, we review the inverse kinematics problem
Tsai andMorganused a higher-dimensional approach to the
andreducetheproblem tosolvingasystemofmultivariate
inverse kinematics problem[24]. In particular, they cast the
polynomials. Section Il introduces matrix polynomials and
problem as eight second-degree equations and solved them
discusses their properties,which are used infinding solutions
numericallyusing polynomial continuation.This isin contrast
ofnon-linearpolynomialequations.InSectionIVwediscuss
with the earlier approaches, where a single polynomial in the
the algorithmfor real timeinversekinematicsforgeneral 6R
tangent of the half-space of one of the joint variables was
manipulators.Wehighlightthesymbolic-numericinterfacein
derived (referred as thelower dimensional approach).Based
1the implementation of the algorithm.The symbolic preprocess-
on their implementation,[24]conjectured that this problem
ning is performed once for a given class of manipulators and the
has at most 16 solutions. The first conclusive proof of the
numeric computation is performed in real time for a given pose
fact that theproblem can have atmost16solutionswasgiven
of the end-effector. The numerical accuracy, implementation
by [19], based on the fact that the remaining 16 solutions to
andperformance of the algorithm are discussed inSectionV.
the 32 degree polynomial in [5] have purely imaginary parts.In Section VI we discuss extensions of the algorithm to general
Finally,[9], [10] gave the exact solution in lower dimensions serial manipulators.A preliminary version of this paper had
by reducing the problem to a 16 degree polynomial. More appeared in [13].
recently, [20], [21] used dialytic elimination to derive a 16
degreepolynomialinthetangentofthehalf-angleofajoint
II.INVERSE KINEMATICS
variable. In [18], [15], examples of a 6R manipulator and
a pose of the end effector are given such that the inverse
A.ProblemFormulation
kinematics problem has 16 solutions.As a result,16 is a tight
bound on the number of solutions.
We use Denavit-Hartenberg formalism,[4], to model a 6R
Algorithms based on the higher as well lower dimensional
manipulator. Each link is represented by the line along its joint
approach have been implemented. It turns out that the problem
axis and the common normal to the next joint axis.In the case
of computingrootsof polynomials ofdegree16 canbeill-
of parallel joints,any of the common normals can be chosen.
conditioned [27]. As a result, in many cases extra precision is
The links of the 6R manipulator are numbered from1 to7.
requiredtoaccuratelycompute the solutionsto theinverse
The base link is 1,and the outermost link or hand is 7.A
kinematics problem.Moreover,implementations based on
coordinate system is attached to each link for describing the
continuationmethodsareratherslowforpractical applications.
relative arrangements among thevarious links.The coordinate
In particular,thebestknown algorithm takes about10seconds
system attached to the ith link is numbered . More details of
on an average of CPU time on an IBM 370 - 3090 using
the model aregivenin[23],[24].The 4× 4 transformation
double precision arithmetic[26],whichfalls short ofwhat is
matrix relating +1 coordinate system to coordinate system
expected of industrial manipulators.
is [23]:
In this paper we present an algorithm and implementation
D!sy!s-
for efficient inverse kinematics for a general 6R manipulator.
!s
CiAi
-Ciμli
aisi
The algorithmmakesuse of symbolicmanipulationused in
A
(1)
0
灿i
i
di
deriving a univariate polynomial and matrix computations.
10
0
0
1
In particular,we use the symbolic formulation presented by
Raghavan and Roth[20].However,the algorithm can also be
where
used along with the formulations given in [1],[9]. The main
s;=sinθ,C=cosθ,0;is the ith joint rotation angle,
contributionofouralgorithmliesinthefactthatweusematrix
operations and reduce the problem to an eigenvalue problem as
opposed to finding roots. These matrix operations correspond
μi = sinαi, 入 = coso， Q:twist angle,
to manipulating matrix polynomials,constructing equivalent
companion matrices and computing their eigendecomposition.
@ : length of link i + 1, d : offset distance at joint i.
The main advantage of this technique lies in its efficiency and
numericalstability.The algorithmsfor computingeigenvalues
For a given robot with revolute joints we are given the a's,
andeigenvectorsofamatrixarebackwardstable'andfast
di's,μui's and X;'s and the pose of the end-effector,attached
implementationsareavailableaspartoflinearalgebrapack-
to link 7. This pose is described with respect to the base link.
ages [2], [7]. This is in contrast with expanding a symbolic
We represent it as:
determinant to compute a degree 16polynomial and thereby,
mxnxqx
computing its roots. For almost all instances of the problem
Ahand
m
ny
fb
we are able to compute accurate solutions using 64 bit IEEE
floating point arithmetic. The average running time of the
0
0
algorithm is 11 milliseconds on anIBMRS/6000.
The problem of inverse kinematics corresponds to comput-
ing the joint angles, 01,02,03, 04, 05 and 06 such that
IAn eigendecomposition algorithm is backward stable if it computes the
exact eigendecomposition of a slightly perturbed matrix.
A1A2A3A4AsA6 = Ahand.
(2)
```

## Page 3

- extraction_method: `ocr_sidecar`
- char_count: `3823`

```text
0S9
IEEE TRANSACTIONS ON ROBOTICS ANDAUTOMATION,VOL.10.NO.5,OCTOBER 1994
The left hand side entries of the matrix equation given aboveisrepresented as:
are functions of the sines and cosines of the joint angles.
Furthermore,this matrix equation corresponds to 12 scalar
equations. Since the matrix formed by the first 3 rows and 3
columns of Ahand is orthonormal, only 6 of the 12 equations
24
are independent.Thus, the problem of inverse kinematics of
general 6R manipulators corresponds to solving 6 equations
A11
A12
A13
2
D
for 6 unknowns.
A21
A22
A23
242
=0
(5)
A11
A12
A21
A22
A23
C405
B.Raghavan andRothSolution
34
We briefly describe the lower dimensional approach de-
scribedbyRaghavan andRoth[20].Theyreduce themul-
tivariate system to a degree 16 polynomial in tan(), such
1
that the joint angle 03 can be computed from its roots.The
where Ajis a3×3matrix andois the 3×3null
other joint angles are computed from substitution and solving
matrix. The entries of Aij are quadratic polynomial in 3.
for some intermediate equations.
Let us represent the left hand side 12 × 12 matrix by ∑. Its
Raghavan and Roth rearrange the matrix equation, (2),as
determinant is a polynomial of degree 24 in c3.It turns out
that (1 + c2)4 divides the determinant and the rest of the 16
A3A4A5=A²A-²AhandA1.
(3)
roots corresponding to r3 component of the inverse kinematics
solution. In the next section we show the equivalence between
this formulation and the non-linear eigenvalue problem. Given
As a result the entries of theleft hand side matrix arefunctions
of 03,04 and 05 and the entries of the right hand side matrix
（equations[20].
are functions of 01,02 and 06.This lowers their degrees and
reduces the symbolic complexity of the resulting expressions.
II.MATRIX POLYNOMIALS
On equating the corresponding entries of the matrix equation,
(3),and after simplification, these equations are expressed in
In this section,we review some literature on matrix polyno-
a linear formulation given as:
mials and present techniquestosolve the non-linear eigenvalue
problem. If Ao,A1,···,Ak are m X m numeric matrices,then
the matrix-valued function defined by
/8485
/S1S2
S4C5
L(X)=E=oAix²
81C2
C182
C485
C4C5
is called a matrix polynomial of degree k.When Ak = I, the
(b)
C1C2
= (P)
84
（4）
identity matrix, the matrix polynomial is said to be monic.
81
C4
More details on matrix polynomials and their properties are
C1
85
given in [6]. In our application we will be dealing with matrix
$2
C5
polynomials in the context of solving non-linear polynomial
C2
1
equations (as shown in (5)). Our main interest is in finding
roots of the polynomial equation
where Q is a 14 x 8 matrix, whose entries are functions of the
P(X) = Determinant(L(x)) = 0.
(6)
parameters of the manipulator and the pose of the end effector.
P is a 14 x 9 matrix, whose entries are linear functions of s3
Asimple solution to thisproblemisexpand the determinant
and c3 and their coefficients are functions of the manipulator and compute the roots of the resulting polynomial. However,
parameters and the pose. The relationship expressed in (4)
）the resulting approach is numerically unstable and expensive
helps us in eliminating four of the five variables.
in practice.
Raghavan and Rothuse8of the 14 equations in(4)to
Let us consider the case when A is a non-singular and well
eliminate the left hand side terms,expressed as functions of 01
conditioned matrix.As a result computation of A1 does not
and θ2,in terms of theright hand side,expressed asfunctions
sintroduceseverenumericalerrors.Let
of 03,04 and 0s.After substituting
L(X) = AL(X), and A= AAi，0 ≤i<k.
2ci
1-x²
L(X) is a monic matrix polynomial. Its determinant has the
same roots as that ofP(入).Let入=入obea root of the
equation
where c=tan(),and taking power products,the system
Determinant(L(x)) = 0.
```

## Page 4

- extraction_method: `ocr_sidecar`
- char_count: `3997`

```text
MANOCHA AND CANNY: EFFICIENT INVERSE KINEMATICS FOR GENERAL 6R MANIPULATORS
651
As a result L(Xo) is a singular matrix and there is at least mk and therefore, has mk eigenvalues. Thus, all the roots of
one non trivial vector in its kernel. Let us denote that m x 1 P(X) correspond to the eigenvalues of C.
Q.E.D.
vector as v. That is
The matrix polynomials have been used to solve general
L(入o)v = 0,
systems of non-linear polynomial equations.More details are
(7)
given in [11],[12]. The relationship between the eigenvalues
where o is amx 1 nullvector.
of C and the roots of P(X)has also been proved using
Theorem 1: Given the matrix polynomial, L(X) the roots
similarity transformations in [6]. Many a time the leading
of the polynomial corresponding to its determinant are the
matrix Akis singular or close to being singular(due to high
eigenvalues of the matrix
condition number). It may still be possible to reduce the
problemtoaneigenvalueproblem usinglinear transformations
0
Im
0
0
(explained in detailinSectionIV-C).However,this technique
0
0
Im
0
may not work at times and in these cases we reduce the
C=
(8)
problem to a generalized eigenvalue problem.
0
0
0
Im
Theorem 2: Given the matrix polynomial, L(X) the roots
-An
-A
-A2
-Ak-1
of the polynomial corresponding to its determinant are the
where o and Im are m × m null and identity matrices, respec-
eigenvalues of the generalized system CX-C2, where
tively.Furthermore, the eigenvectors of C corresponding to
0
o
the eigenvalue 入= Xo are of the form:
0
[ou入u...-]T
C
where v is the vector in the kernel of L(Xo) as highlighted
0
Ak
in (7).
Proof:The eigenvalues of C correspond to the roots of
Im
0
Determinant(C - sI) = 0.
0
0
0
C2=
：
C is a matrix of order mk. Let s = so be an eigenvalue of
0
0
Im
C. As a result there is a non-trivial vector V in the kernel of
-Ao
-A1
-Ak-1
C-soI.Furthermore,we represent V as
where o and Im are m x m null and identity matrices,
V=[...]
respectively.
The proof of this theorem is similar to that of Theorem 1.
and each v; is an m x 1 vector. The relationship between C,
so and V can be represented as
IV.ALGORITHM
0
Im
0
U1
[u1
In this section we describe our algorithm in detail. The
0
0
0
02
U2
initial steps in our algorithm make use of the results presented
0s=
(9)
in [20]. However, we perform symbolic preprocessing and
0
0
k-1
k-1
make certain checks for condition numbers and degeneracy
-Ao
-A1
Luk]
to improve the accuracy of the overall algorithm.The overall
Multiplying the submatrices of C with the vectors in V and
algorithmproceeds in thefollowingmanner:
equating them with the vectors on the right hand side results in:
1)Symbolic Computation:For any class of serial manipu-
lators we perform symbolic preprocessing for simplifica-
U2=S0U1；U3=S0U2;·..Uk=S0Uk-1
tion,minimizing numericalerrors and thecomputation
and
at run time. In particular, we treat the ai's,di's, 入;'s,
μi's and the entries of the right hand side matrix Ahand
-Aou1-A1u2-A203 -..-Ak-1Uk= s0Uk.
as symbolic constants. As a result, express the entries
These relations imply
of the14x9matrixPand14 × 8matrixQ,as
shown in equation (4), as functions of these symbolic
constants. Many geometric properties of manipulators
can be interpreted from the linear algebra structure of
and
thesematrices.Itcorresponds tosymbolicelimination
-(Ao + soA1+ sA2 +..·+ s-1Ak-1 + sIk)1 = 0.
and is performed using the properties highlighted in
[20]. However,it is performed only once for general 6R
Equating the above relation with (7) results in the fact that so
manipulators.An equivalent symbolic elimination can
is a solution of L(X)= O and v1 is a vector in the kermel of
be performed for a serial manipulator with prismatic
L(so) = 0. Thus, every eigenvalue of C is a root of P(X).
and revolute joints.
Since the leading matrix of L(X) is non-singular, P(X) is a
2)Substitution of ManipulatorParameters:Given apar-
polynomialof degreemk.Furthermore,Cisamatrixoforder
ticular 6R manipulator,substitute the numericalvalues
```

## Page 5

- extraction_method: `ocr_sidecar`
- char_count: `4839`

```text
652
IEEE TRANSACTIONSON ROBOTICSAND AUTOMATION,VOL.10,NO.5,OCTOBER 1994
correspondingtothelinklengths,offsetdistancesand
The matrix Q has a special structure.In particular many of
twist angles in the symbolic formulations derived above.its entries are zero and as a result the system of equations,
The substitution results in numerical matrices P and Q，(4), can be expressed as two different systems of equations of
as shown in (4).
the form [20]:
3)Numerical Conditioning: Compute the rank of Q using
SVD(singular value decomposition).If Q has rank 8
$4$5
then this manipulator can have up to 16 solutions for
84C5
any pose of the end-effector.However, the rank may be
C4S5
(@)(a) =(P)]
less than8and as a result we obtain an over-constrained
C4C5
(10)
system. In this case the upper bound on the number of
84
solutions maybelessthan16.Forexample,aPUMA
manipulator has a total of at most 8 solutions for any
85
pose of the end-effector[23].
1C5
4)NumericElimination:Eliminate thevariables0and02
8485
from (4). This elimination is performed by computing a
84C5
/8182
minor of maximum rank of Q and using that minor to
C485
S1C2
represent 01 and 02 as functions of 04 and 05.
C4C5
(Q2)
C182
= (P2)
5)Rank Computation:After eliminating 01and 02,we
(11)
C1C2
obtain a matrix∑.The actual number of rows inE is
C4
82
equal to R =(14-rank(Q))≥6.Take any of the 6 rows
of ∑ (among R) and substitute for sines and cosines of
C5
03,04 and 0s in terms of c3, C4 and T5, respectively. In
case there aremore than 6rows werecommend taking
where Q1,Q2,P1,P2 are 6x 2,8x6,6x 9,8x 9 matrices,
6 distinct linear combinations.
respectively. In particular, we break the set of the 14 equations
6)ReductiontoEigenvalueProblem:Reduce theproblem
into sets of 6 and 8 equations. Q1,Q2 are submatrices of Q
of computing roots of Determinant(E）= 0 to an
1and P1,P2are submatrices of P.
eigenvalue problem. The eigenvalues of the resulting
24 × 24 matrix correspond to the root,23 and the
B.NumericalSubstitutionandRankComputation
correspondingeigenvectors areusedtocompute the
The symbolic preprocessing is performed offline. Given the
values of T4 and cs.Substitute these relations in (4)
manipulator geometry and the pose of the end-effector,the
and(3)tocomputethejointangles01,02and06.
The algorithm also involves clustering eigenvalues to
numerical computations are performed online.In particular,
given theDenavit-Hartenbergparameters of amanipulator,we
accuratelycomputeeigenvalues ofmultiplicitygreater
than one. Depending upon the condition number of the
substitute theai's,di's,入;'sand μi'sintothefunctionsused
to represent the entries of P1,P2,Q1,Q2.These computations
matrices involved,the problem may be reduced to a
are onlyperformedoncefor amanipulator and areindependent
generalizedeigenvalueproblem.
of the pose of the end-effector.As aresult,they are categorized
7)ImprovingtheAccuracy:Computetheconditionnumber
under pre-processing computation. Given the pose of the
of the eigenvalues.In case the condition number is
end-effector, we substitute them to compute the entries of
high, improve the accuracy of resulting solution by
P1, P2,Q1,Q2.Let the corresponding_numerical matrices
Newton's method. The solutions computed above are
(obtained after substitution) be P,P2,Q1,Q2.
the startingpointsforNewton's method andits quadratic
We use singular value decompositions to compute the ranks
convergence gives us high accuracy in a few steps.
of Q1 and Q2 [7]. The singular vectors obtained are also used
Thesesteps areexplainedindetailinthefollowingsections.
to eliminate 01 and 02 from (10) and (11). In particular, let
the singular value decomposition of Q1 be expressed as:
A.Symbolic Preprocessing
√=UE'vT
The algorithm performs symbolic preprocessing for the
where U,E and vT are 6 × 2, 2 × 2 and 2 × 2 matrices,
inverse kinematics solution. It treats the Denavit-Hartenberg
parameters and the entries of Ahand as symbolic constants.
respectively. Initially we compute the singular values, 01,02
These symbolic constants along with thevariables 0;are
of Q.If both the singular values are non-zero,Q has full
usedinthesymbolicderivationoftheequations.Weuse
erank and let Q = Q. If either of the singular values, o is
the computer algebra system, MAPLE,for the derivation
1close to 0.o,we conclude that Q does not have full rank. In
and simplification of the expressions.The coefficients of the
ethiscasewerepresent
equations areused to compute the entries of the matrices P
and Q.As a result,we are able to express the entries of P and
oi
iOi≥e
=
Q as polynomial functions of the symbolic constants. In the
0
3>0
case of P,each entry is of the form βsin(03)+ rcos(03)+ 8，where e is a user defined constant to test the rank deficiency
whereβ. and8 are functions of the symbolic constants.
of the matrix.Furthermorewe compute the elements ofUand
```

## Page 6

- extraction_method: `ocr_sidecar`
- char_count: `3569`

```text
MANOCHA AND CANNY:EFFICIENT INVERSE KINEMATICS FOR GENERAL 6R MANIPULATORS
653
V.and represent
is singular,its condition number is infinity. Let us consider
Q ==10kUiVjk.
the case,when the matrix A is well conditioned. We take the
matrix equation, (14), and multiply it by A-1. Let
Q has the property that a small perturbation does not
∑= Ix² + A-B3+ A-C.
decrease the rank of the matrix. It turns out that this property
 In practice A- B and A-1C are computed by linear equation
has significant impact on the accuracy of the rest of the
solvers. Given E,we use Theorem 1 to construct a 24 × 24
algorithm.We use Q1 for eliminating 01,02 in the system
matrix M of the form
of equations (10) to obtain
W
/84S5
-A-C-A-B
S4C5
It follows from the structure of M that the eigenvalues of
C485
(@)(a)=(P)
M correspond exactly to the roots of Determinant(E)= 0.
C4C5
(12)
Furthermore, the eigenvectors of M, corresponding to the
S4
eigenvaluecghave the structure
85
C5
(15)
T3U
Weperform Gaussian elimination with complete pivoting on
where v is the vector corresponding to the variables in (13).
Q and corresponding row and column operations are carried
Thus, the eigenvectors of M can be used to compute the roots
on the elements of P. Depending on the rank of Q1,whether
of the equations in (13).
0.1 or 2, we obtain 6.5 or 4 equations, respectively, in sines
Inmanyinstances the matrix Ain(14)maybeill-
and cosines of 04,05.
conditioned. One example of such a case occurs,when one of
In a similar fashion we compute the rank of Q2,as repre-
the solution of inverse kinematics has 03 ≈ 180.As a result,
sentedin(1l).Incaseeitherof thesingularvaluesisclose
3=tan(） o.Therefore,A is nearly singular.We
to 0.0,we recompute the matrix Q2 from the singular value
take the matrix equation, (14), and reduce it to a generalized
decomposition of Q2. Otherwise Q2 = Q2.The modified
eigenvalue problemby constructing twomatrices,M1and M2
matrix is used in eliminating 01,02 from (11).Depending on
M=
the rank of Q2,we may obtain anywhere from 2 to8 equations
-B
after elimination.
where o and I are 12 × 12 null and identity matrices, re-
spectively. Furthermore, the roots of Determinant (E）=
C.ReductiontoEigenvalueProblem
0,correspond exactly to the eigenvalues of the generalized
In this section, we reduce the problem of root finding to an
eigenvalue problem M1-t3M2,according to Theorem 2.
eigenvalue problem.Moreover,we exploit the structure of the
The eigenvectors have the same structure as (15).
resulting matrix for efficiently computing its eigenvalues.
Computing the eigendecomposition of a generalized eigen-
We are given a 12 × 12 matrix,∑,whose entries are
value problem is costlier than theeigenvalue problemby
quadratic polynomials in c3.Our problem is to solve the
afactor of 2.5 to3.Inmost cases，we canperform a
system of equations
linear transformation andreduce the problem to an eigenvalue
problem.In particular,we perform a transformation of the
form
0
2
0000000
a3+ b
C3=
(16)
p+&)
where a,b,c,d are random numbers.As a result of this
²
E=
(13)
transformation, (14) transforms into
E=(a²A+ac B+c²C)²+(2ab A+
C4C5
(17)
T4
(ad + bc) B + 2cd C)3 + (b² A+ bd B+ d² C).
0
0
Let A = α² A+ ac B+ c2 C. In most cases A is well
T5
1
10
conditioned. The only exceptions arise when
We express the matrix as
-B
=Ax+Br3+C
(14)
is a singular pencil.A,B.C may have common singular
where A.B and C are 12x12 matrices consisting of numerical pencils. In the latter case, A is ill conditioned for all choices
entries.We compute the condition number of A.If the matrixof a,b,c,d.
```

## Page 7

- extraction_method: `ocr_sidecar`
- char_count: `4722`

```text
654
IEEETRANSACTIONS ONROBOTICSANDAUTOMATION,VOL.10,NO.5,OCTOBER 1994
We try this transformations fora few choices ofa,b,c,d andC.Eigenvector Computation
compute the condition number of A. The cost of estimating
The eigenvector corresponding to a real eigenvalue is com-
condition numberis rather small as compared to computing the
eigendecomposition of the matrix.If Ais well conditioned,
eigenvectorV,we use its structure,(15),to accurately compute
solve for Determinant(E1）= 0 by reducing it to an eigen-
24 and c5 from it.However, due to floating point errors each
value problem. Given T3,apply the inverse transformation to
component of the eigenvector undergoes a slight perturbation.
compute t3. The eigenvectors have the same structure as (15),
Each term of the vector has the same bound on the maximum
except that c3 is replaced by T3.
error occurred due to perturbation [28]. As a result, terms of
maximummagnitudegenerallyhavetheminimumamountof
relative error. We use this property in accurate computation of
V.IMPLEMENTATION
C4 and c5. Given the eigenvector V, let
We have implemented the algorithm on an IBMRS/6000.
|∞3|≤ 1
WehaveusedmanyroutinesfromEISPACKandLAPACKfor
U1=
[|3>1
matrix operations[2].Theseroutines are availableinFortran
andwe interfaced them with our C programs.Many of the
Thus,v1 corresponds to elements of V, whose relative error
algorithmsformatrix computations havebeen specialized to
0is low. 24 and c5 can be computed from v1 by solving for
our application.The details aregivenbelow.
/U1
A.Eigendecomposition
U2
U3
In the previous section we reduced the problem of root
V4
finding to an eigenvalue problem. The 24 × 24 matrix,M,
2
V5
2405
has 24 eigenvalues. However, following the properties of the
V6
symbolicformulation in[20],8of the eigenvalues correspond
1=
(18)
V7
to the roots of the polynomial (1 + α3)4 = 0. In other words,
V8
435
Land-t are eigenvalues of Mof multiplicity 4 each,where
6a
34
↓= √-1. If we transform the variable 3, as shown in (16),
U10
theseeigenvaluesare suitablytransformed.Wemakeuse the
V11
T5
structure ofMalongwiththeQRalgorithmforeigenvalue
U12
1
computation[7].In the double shift QR algorithm we chose
the shift value for the first fewiterations corresponding to
Therefore,r4andt5correspondstoratioof twoterms of
t and -t. It uses at most four iterations of the double shift
t01. Initially, we decide whether|24 |≥ 1 or | x4 |< 1 by
algorithm toreduce theproblem to computing theeigenvalues
comparingthemagnitudeofv1andv2.Asimilar computation
of a 16× 16 matrix.
is perforimed for determining the magnitude of zs. Depending
upon their magnitudes,we tend to use terms of maximum
magnitude such that their ratios correspond to T4 and c5.As
B.ClusteringEigenvalues
a result we minimize the error.
Inmany instances the solution has a root of multiplicity
greater than one. Such cases arise when the manipulator is at
D.Computing All JointAngles
a singular configuration.As such the problem of computing
Given a triple(c3,4,5） corresponding to a solution of the
multiple roots of polynomial equations can be ill-conditioned.
12 equations represented as the 12 × 12 matrix ∑, as shown
In other words the condition numbers for such eigenvalues
in (5). We use these solutions to compute the rest of the joint
can be high and the solution therefore,is not accurate.In
angles. Given the values of s3,C3, 84,C4,85,C5, solve for the
most instances of the problem,we have noticed that there is a
unknowns s1,C1 based on the linear relationship shown in
symmetric perturbation in the multiple roots.For example,let
C3 = α be a root of multiplicity k of the given equation.
(10).Similarly solve the linear system(11) for theunknowns
The floating point errors cause the roots to be perturbed
$2, C2. These five joints angles, 01,.·., 05 are substituted into
(3) to compute 06.
and the algorithm computes k different roots α1,."·,Qk.
Moreover,↑α -α|may be relatively high.Let Qm =
E.Improving theAccuracy
and Qm is very close to the multiple roots.It turns out that
With each eigenvalue,we have the knowledge of its con-
each of the perturbed eigenvalues, Q, can be ill-conditioned;
;dition number and therefore,the accuracy of theresulting
however,the arithmetic mean of the perturbed eigenvalues,solution. If we desire further accuracy,we use these solutions
Qm is well-conditioned [3].We actually verify the accuracyas start points for Newton's iterations on the algebraic equa-
of these computations by computing the condition number
Itionsobtained from(2).Inmostinstances wehavebeen able
of the eigenvalue and the condition number of a cluster of
fto compute the joint angles up to 10 digits of accuracy, by
cluster are available aspartofLAPACK.
convergence).
```

## Page 8

- extraction_method: `ocr_sidecar`
- char_count: `3776`

```text
MANOCHA AND CANNY:EFFICIENTINVERSE KINEMATICS FOR GENERAL 6R MANIPULATORS
655
TABLEI
0.0
0.0
0.0
1.14
0.0
0.30
THE DENAVIT HARTENBERG PARAMETERS OF A 6R MANIPULATOR
0.0
0.0
1.14
-0.0
0.30
0.0
0.0
0.99
0.0
0.09
0.099
0.0
Number
Link length
Offset
Twist angle
0.99
0.0
0.091
0.0
Q2=
0.0
0.099
i
ai
di
0.027
-0.113
0.297
αi
-0.0
1.129
0.0
0.113
0.027
0.0
-0.297
0.0
1.129
1
0.3
0.0
90.0
0.0
1.20
0.068
0.127
0.138
0.062
2
1.0
0.0
1.0
[1.199
0.0
0.126
3
0.0
0.2
006
0.067
0.062
-0.138/
4
1.5
0.0
1.0
The entries of P1 and P2 are functions of s3 and c3.P1 is
5
00
0.0
90.0
an 6 × 9 matrix,
6
0.0
0.0
1.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
.3e-4
0.0
F.Performance
0.0
0.0
0.0
0.0
0.0
3.0
0.0
0.0
0.0
=d
0.0
c3+
We have applied our algorithm to many examples.In
0.0
0.99
1.0
0.0
0.0
0.0
0.0
0.0
0.0
-.003
-.003
0.0
0.0
0.0
0.0
-0.043
particular, we used it on 21 problem instances given in [26]
0.0
Q.0
0.39
and verified the accuracy of our algorithm. All these problems
0.39
0.0
0.0
0.0
0.0
3.0
0.0/
can be accurately solved using double precision arithmetic.
In many cases we are able to compute solutions up to 11-12
0.0
0.0
0.0
0.0
0.0
0.03
0.0
0.0
0.0
digits of accuracy.
0.0
0.017
0.017
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
For most problems, the algorithm takes about 11 millisec-
0.0
+28
0.0
0.0
onds on an average on an IBM RS/6000.About 75-80% of the
0.0
0.0
0.0
0.0
0.0
1.75
0.0
time is spent in the QR algorithms for computing the eigen-
0.0
1.0
-1.0
0.0
0.0
0.0
0.0
6.1e-5
0.0
Q.0
0.11
0.02
0.0
0.0
0.0
0.0
0.017
0.0/
decomposition.Thus,
better algorithmsandimplementations
foreigendecompositioncanimprovetherunningtimeeven
/0.0
0.0
0.0
0.0
1.5
further.
0.0
0.0
0.0
0.2
1.0
0.0
0.0
1.0
In a few cases the algorithm takes as much as 25 millisec-
0.0
0.0
0.0
0.0
0.0
0.0
0.0
onds on the IBM RS/6000.In these instances the matrices
0.0
0.0
0.6
0.0
0.0
0.0
1.9
0.2
0.0
0.0
-0.20
A,B.C in (14) are ill-conditioned and have singular pencils.
0.0
0.0
1.5
0.0
0.10
0.017
0.0
0.0
0.044
0.0
0.0
0.0
0.0
0.03
As a result wereduce the resultingproblem to a generalized
←1.29
0.0
eigenvalue problem,which slows down the algorithm.
0.0
3.21
0.0
0.0
-0.6
0.0
0.68
Example:Let us consider the manipulator presented in[26]
Similarly, P2 is a 8 × 9 matrix
along with a pose of the end effector.This is problem 6 in
0.0
0.0
0.0
0.0
0.0
1.5
0.0
0.0
0.0
[26]and corresponds to a slight variation of the manipulator
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
presented in[15].For this configuration the problem of inverse
0.0
1.0
1.0
0.0
0.0
0.0
0.0
0.0
0.0
kinematics has 16 real solutions. The robot parameters are
P2=
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.017
0.0
given in Table I.
0.0
0.026
0.0
3.5e-3
+
0.0
0.0
0.0
0.0
0.0
The position and orientation of the end effector and is given
0.0
0.2
0.2
0.0
0.0
0.0
0.0
1.5
0.0
by the matrix
0.0
1.29
3.21
0.0
0.0
0.0
0.0
0.6
0.0
p.0
0.017
-6.9e-3
0.0
0.0
0.0
0.0
0.11
0.0/
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
/-0.7601
0.6416
0.1022
1.1401
0.0
0.0
0.0
0.0
0.0
-1.5
0.0
0.0
0.1333
0.0
0.9910
0.0
0.0
Ahand =
0.0
0.0
0.6359
0.0
0.0
0.0
0.0
0.0
0.02
0.7669
0.0
0.0855
0.0
0.0
1.0
-1.0
0.0
0.0
0
0
0.0
0.0
0.0
0.0
0
1
0.0
0.02
0.2
s3+
0.0
0.0
0.0
0.0
1.5
0.0
0.0
0.044
0.017
0.0
0.0
0.0
0.0
-3.5e-3
0.0
0.0
0.01
0.0
0.0
0.0
0.0
0.0
0.023
0.0
After substitution into the symbolic matrices,we obtain
Q.0
3.29
1.21
0.0
0.0
0.0
0.0
0.6
0.0/
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
1.0
0.0
0.0
0.0
0.0
0.03
0.0
0.0
0.0
3.5e-3
(-1.140
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0910
0.990
0.017
0.0
0.0
0.017
0.0
0.0
0.0
0.0
0.0
Q=
0.0
0.684
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
0.297
0.027
1.0
0.0
0.0
0.1
0.0
0.0
0.0
-3.49e-3
0.0
0.0
0.112
0.40
0.0
0.0
0.4
0.0
0.0
-3.0
0.0
0.0
0.110
1.377/
0.02
0.0
0.0
0.11
0.0
0.0
0.01
0.0
0.0
```

## Page 9

- extraction_method: `ocr_sidecar`
- char_count: `5006`

```text
656
TABLE II
TABLE III
EIGENVALUES AND THEIR CONDITION NUMBERS
THE JOINT ANGLES CORRESPONDING TO THE SOLUTIONS
Number
Eigenvalue
Condition number
01
02
03
04
05
06
1
3679.99
9.32215
1
-96.28
-6.27
179.96
38.48
52.55
-39.40
2
-123.591
11.3508
2
-120.78
172.33
-179.07
31.33
-146.71
142.82
3
-35.0237
7.71049
3
88.67
-176.72
-176.72
-63.24
157.19
140.43
4
-50.794
8.82256
4
113.84
5.30
-177.74
-55.92
-62.98
-43.37
5
-3.45709
10.4068
5
-178.12
108.19
-147.73
-5.69
-164.67
179.58
6
3.33357
9.10936
6
168.32
-103.89
146.60
-17.24
-171.87
98.16
7
-1.56894
7.09899
7
-12.94
-105.09
-114.97
3.02
7.41
-79.42
8
1.48377
6.83255
8
2.51
108.07
112.04
-10.52
0.00
-0.10
9
-0.673961
6.83255
9
2.51
108.07
-67.95
-169.47
179.99
179.89
10
0.637372
7.09899
10
-12.94
-105.09
65.02
176.97
172.58
100.57
11
-0.299978
9.10936
11
168.32
-103.89
-33.39
-162.75
-8.12
-81.83
12
0.289261
10.4068
12
-178.12
108.19
32.26
-174.30
-15.32
-0.41
13
0.0285521
7.71049
13
88.67
-176.72
3.27
-116.75
22.80
-39.56
14
-0.00027174
9.32215
14
-96.28
-6.27
-0.03
141.51
127.44
140.59
15
0.00809121
11.3508
15
-120.78
172.33
0.92
148.66
-33.28
-37.17
16
0.0196874
8.82256
16
113.84
5.30
2.25
-124.07
-117.01
136.62
The matrices Q1and Q2have no singular values close to
VI.GENERALSERIALMANIPULATORS
zero. In other words they are full rank matrices. As a result
The techniques presented have been extended to all serial
after numerical elimination we obtain a 6 x 9 matrix ∑. is
manipulators with a finite number of solutions by making use
convertedintoamatrixpolynomialusing the transformation
of the matrix structures [14].The joints may be prismatic
T3 = tan(） and obtaining the 12 × 12 matrix E",expressed
Or revolute.In particular,Raghavan and Roth have shown
asamatrixpolynomialint3.Theestimatedconditionnumber
that for many cases of manipulators with six joints(revolute
of the leading matrix is 5000.0.2 As a result, we reduce it
or prismatic) the problem of inverse kinematics reduces to
to an eigenvalue problem of a 24 × 24 square matrix. The
finding roots of a univariate polynomial [21]. It involves taking
eigenvalues are computed using LAPACK routines.The real
suitableminors of matrix andreduction toan eigenvalue
eigenvalues and their condition numbers are given in Table I1.
problem.
Thus, we see that all the 16 eigenvalues are real. Fur-
thermore, they are computed up to 15 digits of accuracy.
VII.CONCLUSION
Thisfollowsfromthefactthatthemachineconstantfor
IEEE floating point arithmetic is of the order of 10-16 and
In this paperwepresented anefficient algorithmforinverse
themaximumconditionnumberisoftheorderof11.Asa
kinematics of a6Rmanipulatorofgeneralgeometry.Thealgo-
result, the eigenvalues have a relative error bounded by 10-15
rithmsperforms symbolicpreprocessing,matrixcomputations
Giventheeigenvalues,therestofthealgorithminvolves
and reduces the problem to computing the eigendecomposition
computation of rest of the corresponding eigenvectors and
of a matrix. The numerical accuracy of the operations used
joint angles.Let's illustrate the process for the first eigenvalue,
in the algorithm is well understood.For most instances of
T3 = 3679.99. As a result,
the problem the solution can be accurately computed using
double precision arithmetic.The algorithm has been tested on
s3=0.00054348,
c3=-0.999999.
a variety of instances and the average running time is 11 ms
on an IBM RS/6000. We believe that this algorithm gives us a
Since |z3|>1,we make v1 equal to the last 12 elements of
level of performance expected of industrial manipulators.This
V the eigenvector, as shown in (18).Analyzing the elements
approach can be directly extended to all serial manipulators
of v1 results in 丨c4 |< 1 and|c5 < 1. Elements of
with a finite number of solutions.
maximum magnitude of v1 areused to compute T4 and c5
to the best possible accuracy. It results in 24 = 0.34907 and
ACKNOWLEDGMENT
5 = 0.49368. These are used to compute s1,s2,cl,c2,s6,c6
We are grateful toJ. Demmel for productive discussions on
by solvinga system of linear equations.
matrix computations.
Given the sines and cosines of the joint angles, s; and Ci,
their accuracy is improved by using a few iterations of the
REFERENCES
Newton's method. As a result, it is possible to obtain solutions
to 12 digits of accuracy on this example.The 16 solutions for
[1] H. Albala and J. Angeles,“Numerical solution to the input output
displacement cquation of the general 7R spatial mechanism,” inProc.
this position and orientation of the end-effector are given in
FifthWorld Cong.TheoryMach.Mechanisms,1979,pp.1008-1011.
Table III.
[2]E.Anderson,Z.Bai,C.Bischof,J.Demmel,J.Dongarra,J.Du Croz,A.
Greenbaum,S.Hammarling,andD.Sorensen,LAPACKUser's Guide,
Release 1.0,Philadelphia,PA: SIAM,1992.
2In practicewe have been ableto linearizematrixpolynomials withleading
[3]Z. Bai, J. Demmel, and A. McKenney,“On the conditioning of the
matrices of condition number up to1e05 to eigenvalue problems.
nonsymmetric eigenproblem:Theory and software,"Computer Science
```

## Page 10

- extraction_method: `ocr_sidecar`
- char_count: `4288`

```text
657
Dept.Technical Report 469,Courant Institute,New York,NY,October,
of general geometry," in Int.Symp.Robotics Res.,pp.314320,Tokyo,
1989 (LAPACK Working Note #13).
1989.
[4]J. Denavit and R. S. Hartenberg,“A kinematic notation for lower-pair
[21] M.Raghavan and B.Roth,“Inverse kinematics of the general 6R
mechanisms based upon matrices,"J.App.Mechanics,77,pp. 215221,
manipulator and related linkages,” Trans. ASME J. Mech.Des.,to
1955.
appear.
[5]J. Duffy and C.Crane,“A displacement analysis of the general spatial
[22]B.Roth,J. Rastegar,and V. Scheinman,“On the design of computer
7R mechanism,Mechanisms Mach.Theory,vol.15,pp.153-169,1980.
controlledmanipulators,”in On the Theory and Practice of Robots and
[6]1. Gohberg,P.Lancaster,and L.Rodman,Matrix Polynomials.New
Manipulators,First CISMIFToMM Symposium,1973,pp.93113.
York: Academic Press,1982.
[23]M.W.Spong and M.Vidyasagar,Robot Dynamics and Control.New
[7]G.H. Golub and C.F.Van Loan,Matrix Computations.Baltimore,
York: John Wiley and Sons,1989.
MD: John Hopkins Press, 1989.
[24] L. W. Tsai and A. P. Morgan,“Solving the kinematics of the most
[8]S. A. Hayati.“Robot arm geometric link calibration,” in IEEE Contr.
general six and five-degree-of-freedommanipulators by continuation
Decision Conf,1983,pp.1477-1483.
methods,”Trans.ASME J.Mech.Transmissions Automat.Des.,107,
[9]H.Y.Lee and C.G.Liang,“Displacement analysis of the general spatial
pPp.189-200,1985.
7-link 7R mechanism,"Mechanisms and Machine Theory,vol.23,no.
[25] W.K. Veitschegger and C. Wu,"A method for calibrating and com-
3,Pp.219226,1988.
pensatingrobotkinematic errors”inIEEEConfRoboticsAtomtp
[10]H.Y. Lee and C.G.Liang,“A new vector theory for the analysis
3943, 1987.
of spatial mechanisms,”Mechanisms Mach.Theory,vol. 23,no.3,pp.
[26]C.Wampler and A.P.Morgan,“Solving the 6R inverse position problem
209-217,1988.
usinga generic-case solution methodology,”MechanismsMach.Theory.
[11]D.Manocha,“Algebraic and numeric techniques for modeling and
vol. 26,no. 1,Pp. 91106,1991.
robotics,”Ph.D. thesis,Computer Science Division,Department of
[27]J. H.Wilkinson,“The evaluation of the zeros of ill-conditioned polyno-
Electrical Engineering and Computer Science,University of Califomia,
mials-Parts i and i, Numer. Math., vol. 1, pp. 150180, 1959.
Berkeley，May 1992.
[28]J. H.Wilkinson,The algebraic eigenvalue problem.Oxford:Oxford
[12]D.Manocha,Solving systems of polynomial equations,IEEE Comput.
University Press, 1965.
Graph.Applicat.,Special Issue on Solid Modeling,Pp.4655,March
1994.
[13]D.Manocha and J.F. Canny,“Real time inverse kinematics of general
6R manipulators,”in Proc.IEEE Conf.Robotics Automat.,pp.383-389,
1992.
Dinesh Manocha received the B.Tech.degree in
[14]D. Manocha and Y. Zhu,“A fast algorithm and system for inverse
computer science from Indian Institute of Tech-
kinematics of general serial manipulators,，"inProc.IEEEConf.Robotics
nology, Delhi, in 1987, and the M.S. and Ph.D.
Automat.,1994.
degrees in computer science from the University of
[15]R.Manseur and K. L. Doty,“A robot manipulator with 16 real inverse
California at Berkeley in 1990 and1992,respec-
kinematic solution set,"Int.J. Robotics Res., vol. 8, no. 5,pp. 7579,
tively.He received an Alfred and Chella D. Moore
1989.
fellowship and an IBM graduate fellowship in 1988
[16]B.Paden and S.Sastry,“Optimal kinematic design of 6Rmanipulators,”
and 1991,respectively,and a junior faculty award in
Int.J.Robotics Res.,vol.7,no.2,pp.43-61,1988.
1992. He is currently an assistant professor of com-
[17]C.J.Paredis and P.K.Khosla,"An approach for mapping kinematic task
puter science at the University of North Carolina at
specification into a manipulator design,”in FifthInt. Conf.Advanced
Chapel Hill. His research interests include geometric
Robotics,Pisa,Italy,June 1991.
and solid modeling, virtual environments,geometric constraint systems and
[18]D.Pieper,“The kinematics of manipulators under computer control,”
symbolic and scientific computation.
Ph.D. thesis,Stanford University，1968.
[19]E. J. F. Primrose, “On the input-output equation of the general 7R-
mechanism,”Mechanisms and Machine Theory,vol. 21,pp.509-510,
1986.
[20] M.Raghavan and B.Roth,“Kinematic analysis of the 6R manipulator John F. Canny photograph and biography unavailable.
```

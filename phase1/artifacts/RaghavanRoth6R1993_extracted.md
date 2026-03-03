# Extracted Text: RaghavanRoth6R1993.pdf

## Page 1

- extraction_method: `direct`
- char_count: `5641`

```text
fVl. Raghavan1 
B. Both 
Mechanical Engineering Department, 
Stanford University, 
Stanford, CA 94305 
Inverse Kinematics of the General 
6R Manipulator and Related 
Linkages 
This paper elaborates on a method developed by the authors for solving the inverse 
kinematics of a general 6R manipulator. The method is shown to be applicable to 
determining the joint variables associated with all series-chain manipulators and 
closed-loop linkages constructed in a single loop with revolute, prismatic, or cylindric 
joints. The method is shown to yield a single polynomial, of minimum degree, in 
terms of just one of the joint variables. Once the roots of this polynomial are found, 
the remaining variables are then usually determined from linear sets of equations. 
It is shown that this method works equally well for general geometries and for special 
geometries such as those chararcterized by intersecting or parallel joint axes. 
Introduction 
In this paper we describe a problem formulation and elim
ination technique which can be applied to a large class of 
manipulators and closed-loop mechanisms in order to deter
mine: (a) for a manipulator, all possible joint-displacement 
values corresponding to a specified end-effector pose, (b) for 
a closed-loop mechanism, all possible joint-displacement val
ues corresponding to a specified input displacement. 
This paper uses the solution method developed by the au
thors originally for solving the inverse kinematics of 6R series-
chain manipulators of general geometry, Raghavan and Roth 
(1989). We also rely on additional proofs presented by Rag
havan (1990), and an extension of the original method to ma
nipulators with prismatic joints, as given by Raghavan and 
Roth (1990). 
The formulation we use produces a system of multivariate 
polynomials in terms of the unknown joint displacements. The 
method we have developed for the solution of these equations 
brings about the elimination of all the joint variables except 
for one, which we call the suppressed variable. The method 
always yields a single "characteristic" polynomial, of mini
mum degree, in terms of just this single suppressed joint vari
able. Once the roots of this polynomial are found, the variables 
that were eliminated can be determined from linear sets of 
equations. It is shown that this method works equally well for 
general geometries and for special geometries such as those 
characterized by intersecting or parallel joint axes. 
Both the so-called inverse manipulator-kinematics problem 
and the closed-loop linkage analysis problem, problems (a) 
and (b) above, have been the subjects of many previous re
search publications. The fact that both problems are in fact 
connected seems to have first been published in Roth, Rastegar, 
and Scheinman (1973). It is now widely understood that if we 
'Present address: Power Systems Research Department, General Motors Re
search Laboratories, Warren, MI 48090-9055. 
Contributed by the Mechanisms Committee and presented at the Design Tech
nical Conference, Chicago, IL, Sept. 16-19, 1990, of THE AMERICAN SOCIETY 
OF MECHANICAL ENGINEERS. Manuscript received March 1990. Associate Tech
nical Editor: J. M. McCarthy. 
consider the known position of the input link of a closed-loop 
linkage as the known position of a manipulator hand, the 
displacement (or position) analysis of any n-link closed-loop 
linkage is identical to the inverse kinematics of a corresponding 
«-link manipulator (including the end-effector), and vice versa. 
This idea is now used extensively, see for example the book 
by Duffy (1980). 
Previous studies of linkage analysis can be traced back to 
the last century and are too numerous to list here. Interested 
readers can consult, for example, Duffy (1980), Yang and 
Freudenstein (1964), and De Groot (1970). The inverse ma
nipulator-kinematics problem was first discussed in Pieper 
(1968). A bibliography for the research into the inverse 6R 
problem can be constructed from the papers by Albala and 
Angeles (1979), Duffy and Crane (1980), Tsai and Morgan 
(1985), Lee and Liang (1988a and 1988b) and Raghavan and 
Roth (1989). The inverse kinematics of series-chain manipu
lators with special geometries and with prismatic joints have 
been treated systematically in, for example, Pieper and Roth 
(1969), Duffy (1980) and Raghavan and Roth (1990), and in 
a more ad hoc manner in numerous analyses of specific com
mercial and experimental manipulators [see, for example, text
books such as Paul (1981) and Craig (1989).] 
Problem Formulation 
In order to understand how to study any single-loop open 
or closed loop system with our method, it is important to 
understand how it is applied to the 6R manipulator. To ac
quaint the reader we repeat the essential elements of our de
velopment for the 6R manipulator problem in Raghavan and 
Roth (1990). Namely, using exactly the same variant of the 
Denavit and Hartenberg nomenclature as in Tsai and Morgan 
(1985), the links of the 6R manipulator are numbered from 1 
to 7, the fixed or base link being 1, and the outermost link or 
hand being 7. A coordinate system is attached to each link to 
facilitate a mathematical description of the linkage and the 
relative arrangement of the links. The coordinate system at-
502 / Vol. 115, SEPTEMBER 1993 Transactions of the ASME 
Copyright © 1993 by ASME
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 2

- extraction_method: `direct`
- char_count: `4630`

```text
Ar il) 
tached to the rth link is numbered i. The 4x4 transformation 
matrix relating coordinate systems i + 1 and / is as follows: 
(Ci -Sih SM afit 
Si CjKi -Cim an 
0 in X, d, 
0 0 0 1 
where 
S( = sin0;, Cj = cos0;, 
A,- = cosa,-, IXJ = sina,-, 
a-, is the length of link i + 1, 
a-, is the twist angle between the axes of joints / and ;' + 1, 
di is the offset distance at joint i, 
dj is the joint rotation angle at joint /. 
The closure equation for the 6R manipulator is the following 
matrix equation 
/M^M-kMe =Aand (2) 
y4hand is the 4x4 transformation matrix describing the Carte
sian coordinate system 7, attached to the hand (or last link) 
with respect to coordinate system 1, attached to the base link. 
The entries of this matrix are known because the hand coor
dinates in the goal position are specified. The left-hand side 
of the above matrix equation describes coordinate system 7 
with respect to coordinate system 1, in terms of the relative 
arrangements of the intermediate coordinate systems. The 
quantities «,-, dj, A,-, /*,-,; = 1,. . . ,6, appearing in the matrices 
on the left-hand side of Eq. (2) are all known. The unknown 
quantities are 0,, / = 1, . . . ,6, and the above matrix equation 
must be solved for them. This is the inverse kinematics problem 
for the 6R series manipulator. 
Derivation of p and I 
The matrix Eq. (2), is equivalent to 12 scalar equations. Of 
these only 6 equations are independent, because the submatrix 
comprised of the first 3 rows and columns is orthogonal. We 
perform operations on these multivariate equations and elim
inate all but one variable, thus reducing the problem to the 
solution of a single equation in one variable. This is in the 
spirit of Gauss Elimination applied to systems of linear equa
tions, except that the equations in this problem are nonlinear. 
Let Ahaad be equal to 
Equation (2) may be rewritten as 
A-iAiA^A^A^A^&Al1 
'lx 
ly 
lx 
•.0 
mx 
my 
mz 
0 
nx 
ny 
nx 
0 
Px\ 
Py 
Pz 
1/ 
(3) 
We do this so as to move du 02, and 06 to the right-hand side. 
This lowers the degrees of the equations and also reduces their 
complexity. 
There is another preliminary concern which simplifies mat
ters. It is useful to remember that an A) matrix can be written 
as the product of two matrices: As = AivAis, in which AjS 
contains only the three structural Denavit and Hartenberg pa
rameters, and Aiv contains only the joint variable. We have 
for a revolute joint and prismatic joint, respectively: 
Aiv = 
A,„=\ 
ICi 
Si 
0 
\0 
a 0 
0 
\Q 
-Si 
Ci 
0 
0 
0 0 
1 0 
0 1 
0 0 
0 0\ 
0 0 
1 0 
0 1/ 
°\ 0 
di 
1/ 
Ajs - 
a 0 
0 
*0 
ICi 
Si 
0 
*0 
0 0 
A,- - m 
Hi A; 
0 0 
-Sih 
c,X, 
/*/ 
0 
a\ 
° * 
1/ 
SiHi 
- CiHi 
X; 
0 
» 
afii 
aft 
0 
1 
Substituting A, = AivAis for the matrix at one end of the 
equation, makes it simple to have a link's motion parameter 
isolated from its structural parameters; the advantage is that 
the motion parameter then appears in a simple form. So we 
premultiply (3) by A^ in order to simplify the structures of 
the 02 terms. The resulting equation is then 
-l 
A2sAiA^As --A2M-1 AhaadA6 (4) 
When the matrix multiplications are carried out, Eq. (4) has 
the form 
/(03>04,05) (03,04,05) (03,04,05) (^3»^4.^5)\ 
l(03,04,05) (03,04,0s) (03,04,05) (#3.04 A) ] 
(04,05) (04.05) (04,05) (04,05) 
0 0 0 1 
/(01,02,06) (01,02,06) (01,02) (0.,02)\ 
01,02,06) (01,02,06) (0!,02) (01,02) 
[(01,02,06) (01,02,06) (01,02) (01,02) J 
0 0 0 1 
(5) 
Equation (5) shows the variables appearing in the individual 
entries in Eq. (3). The 6 scalar equations obtained from col
umns 3 and 4 of Eq. (3) are devoid of 06. These equations 
though linearly independent are governed by the constraint 
that the magnitude of the column 3 vector is unity. We work 
with these 6 equations with the goal of eliminating 4 of the 5 
variables so as to obtain a univariate polynomial which will 
vanish at their common zeros. The 6 equations are: 
[ci s2 0\ I 
\sf -c2 0 n= 0 
,6 o 1/ \o 
where h = | h2 
(6) 
(7) 
\h-
f\=C4gi+SAg2 + Cli 
fl= - X3 (S4gl - Oft) + /*3#3 
h = M3 (stg! -c4g2)+ X3g3 + di 
ri = cAnii+S4m2 
r2 = - X3 {s4mi - cAm2) + 1x^3 
r3 = H3 (54/«i - c4m2) + X3/M3 
gj = c5as + «4 
g2= -ss\4a5 + ix4d5 
g3=S'5^4«5 + X4C?5 + rf4 
mi=SSH5 
m2 = c5X4/i5 + /i4X5 
m-i = - CiHAy-5 + X4X5 
Journal of Mechanical Design SEPTEMBER 1993, Vol. 115 / 503 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 3

- extraction_method: `direct`
- char_count: `3637`

```text
h^CiP + SiQ-Oi 
h2 = -Xiisxp-c^+niir-di) 
h3 = ixi(siP-Ciq)+\i(r-di) 
«2= -\i(SiU-CiV)+^\W 
«3 = jX\ (S\U -C\V)+\\W 
P = -lifl6- (mxii6 + nx\6)d6 + px 
q = -lya6- (myix6 + ny\6)d6 + py 
r= - 4«6 - (mzH(, + nz\(,)d6 + pz 
u = mxn6 + nx\ 
v = myij.6 + ny\f, 
w=mznt + nz\e 
Henceforth we will refer to Eqs. (6) and (7) by the vectors 
p and T respectively. By this we mean 
c2 s2 
p=\s2 
10 
and 
Id s2 0\ 1 0 
1=1 J2 -c2 0 n= 0 -X2 
^0 0 \j \0 M2 
Let p^ p2, and lu l2, h be respectively the components of p 
and T. We now proceed to eliminate 4 of the 5 variables in p 
and T by exploiting the structure_of the ideal generated by the 
component equations of p and 1. 
The Ideal of p and 1 
It is noteworthy that each of/1(/2, f3, ru r2, r}, is a linear 
combination of the terms 5455, s4c5, c^s5, c4c5, s4, c4, s5, Cs, 1. 
Similarly each of h\, h2, hit nt n2, n3 is a linear combination 
of the terms suCi, 1. Equations (6) and (7) taken together may 
be written in matrix form as: 
{A) 
S4S5 
•S4C5 
C4S5 
C4C5 
s4 
c4 
s5 
cs 
1 
= (B) 
SiS2 
SiC2 
ClS2 
C\C2 
Si 
Ci 
Si 
c% 
(8) 
where A is a 6 X 9 matrix whose entries are linear combinations 
ofs3C3, l,andBisa6 x 8 matrix whose entries are all constants. 
The ideal generated by a set of polynomials qu q2, . . . , qn 
in the variables xlt x2 . . . , x,„ is the set of all elements of the 
form gift + q2$2 + . . . + q3r, where ft, /32, . . . , ft- are 
arbitrary elements of the set of all polynomials in x\, x2, . • • , 
The ideal generated by the component equations of p and 
T has the following interesting properties: 
Property 1: p • p has the same power products as p and I 
(i.e., Eq. (8)). By power products we mean "terms" (e.g., the 
power products of the polynomial 5x2y + 3xz + 9y + 4z = 
0 are jry, xz, y2 and z.) Therefore the equation p • p may be 
written in the form 
(M) 
54S5 
S4C5 
C4S5 
C4C5 
.54 
c4 
s5 
c$ 
1 
^ 
(N) 
STS2 
SlC2 
CiS2 
C1C2 
Sl 
Ci 
s2 
c2 
where Mis a 1 X 9 matrix whose entries are linear combinations 
of sit c3, 1 and Nis a 1 x 8 matrix whose entries are constants. 
Property 2: p«T has the same power products as p and !._ 
Property 3: p xl has thesame power products as p and T. 
Property 4: (p-p)I- (2p«l)p has the same power products as 
p and T. 
A rigorous justification of properties 1-4 is presented in 
Raghavan and Roth (1989). 
We now have the following set of linearly independent equa
tions all of which have the same power products: 
Vector No. of Scalar Equations 
P 
I 
P • P 
P-l 
p x I 
(p • p)l - (2p • T)p 
3 
3 
1 
1 
3 
3 
(Total) 14 
These 14 equations may be written in matrix form as 
r 
(P) 
<. 
S4SS 
S4C5 
C4S5 
C4C5 
s4 
c4 
S5 
c5 
1 
= (Q) 
S\S2 
S\C2 
cxs2 
C\C2 
S\ 
C\ 
s2 
c2 
(9) 
where P is a 14 x 9 matrix whose entries are linear combi
nations of s3, c3, 1 and Q is a 14 X 8 matrix whose entries are 
all constants. We now proceed to eliminate variables sequen
tially from the above equations. 
Elimination of 0! and 02 
We use any 8 of the 14 equations in Eq. (9) to solve for the 
8 right-hand side terms containing dx and 02 in terms of the 
left-hand side which is a function of 03, 04, and 05. We use 
these to eliminate terms containing di and 02 from the remaining 
6 equations, which then take the form: 
*~ S4SS 
s4c5 
CAS5 
C4C5 
(E) s4 =0 (10) 
c4 
ss 
Cs 
1 
504 / Vol. 115, SEPTEMBER 1993 Transactions of the ASME 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 4

- extraction_method: `direct`
- char_count: `4590`

```text
where E is a 6 x 9 matrix whose entries are linear combinations 
of s-i, c3, 1. 
Elimination of 04 and ds 
We make the following substitutions in Eq. (10): 
S4-| 
2XA I-4 
1+4** i+4 >Q ,Ss~ 2x5-
1+4 ,C5' 
1-4 
'i+4 
where x4 = tan M I ,x5'= tan I ~ 
We then multiply each equation by (1 +4) and (1+4) 
to clear denominators. Equation (10) then takes the form 
44 
X4X5 
4 X4X5 
(E') X4X5 =0 
x4 
4 
x5 
1 
where E' is a 6 X 9 matrix whose entries are linear combi
nations of 53, c3, 1. We make the following substitutions in 
Eq. (11): 
(11) 
*3~ 
2xi „ i-4 
i+4' ^1+4 
We multiply the first 4 scalar equations in Eq. (11) by (1 + 
4) to clear denominators. The resulting equation is of the form 
44 
4xs 
x% 
X4X5 
(E") X4X5 =0 (12) 
4 
x5 
1 
where E " is a 6 x 9 matrix. The entries in the first 4 rows of 
E" are quadratic polynomials in x3. The entries in the last 2 
rows are rational functions of X3, the numerators being quad
ratic polynomials in x3, the denominators being (1 + 4)- It 
is noteworthy that the determinant of the 6x6 array com
prised of any set of 6 columns of E " is always an 8th degree 
polynomial and not a rational function. This fact is proved 
rigorously in Raghavan and Roth (1989). 
We now eliminate x4 and x5 dialytically [see Salmon (1885)] 
as follows: Multiplying Eq. (12) by x4, we get the following 
equation 
X4X5 
4^5 
4 xixj 
(E") 4x5 =0 (13) 
4 
x$x\ 
X4X5 
X4 
Equations (12) and (13) taken together may be written in matrix 
form as: 
xixj 
x%xs 
4 
^4X5 
A4X5 
4 X4X1 
E" • 0 
0 E" = 0 (14) 
x, 
4 
xs 
1 
Equation (14) constitutes a set of 12 linearly independent equa
tions in the 12 terms x\x\, xlx5, x\, x\x\, xjx5, x\, xAx\, x4x5, 
x4, x\, x$, 1. This is clearly an overconstrained linear system. 
In order for this system to have a nontrivial solution, the 
coefficient matrix must be singular. The determinant of the 
coefficient matrix is a 16th degree polynomial in x3. The roots 
of this polynomial give the values of x3 corresponding to the 
16 solutions of the inverse kinematics problem. For each value 
of X3 thus obtained, the corresponding value of 03 may be 
computed using the formula 03 = 2tan_1x3. 
The Remaining Joint Variables 
For each value of 03, we may compute the remaining joint 
variables as follows: We substitute the numerical value of 03 
in the coefficient matrix of Eq. (14). We then use 11 inde
pendent members of Eq. (14) to solve for the 11 terms x\x\, 
X4X5, X4, X4X5, X4X5, X4, X4X5, X4X5, X4, X$, X5. The numerical 
values of Xa, and x5 may be used to compute 64 and 85. We then 
substitute numerical values for 83, 84, and 8$ in Eq. (9). We 
use 8 linearly independent members of the resulting equation 
to solve for Sis2, SiC2, CiS2, CiC2, Si, clt s2 and c2. We then use 
the numerical values of Si and C\ to obtain a unique value for 
81. Similarly, 82 may be computed using the numerical values 
of s2 and c2. Finally, substituting values for 8U 82, 03, 84, and 
05 in the (1, 1) and (2, 1) elements of the following equation 
A6 = A5'A4 lA3lA2 XAf [Ahand (15) 
yields 2 linear equations in s6 and c6. After solving for s6 and 
c6 we may use their values to determine a unique value for 86. 
Characteristic Polynomial 
It is important to notice that in the foregoing solution method 
all the joint variables are obtained from linear equations once 
the suppressed variable, 03, is known. The 16th degree poly-
E" 0 nomial we get by expanding the determinant n = 0, 
determines the number of solution sets for each hand position. 
We call this polynomial the characteristic polynomial, since it 
characterizes the number of possible solution sets. The max
imum number of possible values for the variable 03 is 16, i.e., 
the degree of the characteristic polynomial. In general not all 
of the characteristic polynomial's roots will be real, and the 
actual number of solutions will often be much less than 16. If 
the specified hand position is not reachable all of the roots 
will be imaginary (i.e., the roots are all complex numbers), 
signifying that it is not physically possible for the manipulator 
to place the hand in the specified position. 
Although the number of real roots of the characteristic pol
ynomial usually determines the actual number of solution sets, 
Journal of Mechanical Design SEPTEMBER 1993, Vol. 115/505 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 5

- extraction_method: `direct`
- char_count: `8024`

```text
of the variables that were eliminated during the process of 
obtaining the characteristic polynomial, this is not always the 
case, and for certain types of manipulators the number of 
solution sets is actually double the number of real roots of the 
characteristic polynomial. This occurs when some joint vari
ables become multivalued with respect to the suppressed vari
able. Mathematically this occurs whenever we have only a 
single linear equation in either the sine or the cosine of an 
angle. For an angle to be a single-valued function of the sup
pressed variable it is necessary for there to be two linear con
ditions on its sine and/or cosine, or a single linear condition 
on the tangent of its half-angle. • 
Alternative Equations 
The physical meaning of our equations p and T are imme
diately obvious from (4). Let 2' represent a coordinate system 
which differs from the Denavit and Hartenberg 2-system by 
having its x-axis rotated parallel to the x-axis of the 3-system. 
Clearly p represents the coordinates in the 2' system of a vector 
from the origin of the Denavit and Hartenberg coordinates in 
the 2 system to the origin in the 6 system, and I represents the 
directions of a unit vector parallel to the z-axis in the 6 system 
as measured along axes parallel to the 2' system. The right-
and left-hand sides of p and I simply give the measures of these 
vectors in terms of the variables denoted by the subscripts on 
the two sides of (4); clearly these vectors must have equal 
measures if we transform successive coordinates moving in
ward toward the base, or moving out toward the hand and 
then through the base outward toward the hand. 
There is nothing special about the choice of the 2, 3, and 6 
coordinate systems. The all-important power product prop
erties 1-4 are a function of the structure of the A, matrices, 
see Raghavan (1990). So it is possible to change (4) in a cyclic 
manner and still maintain the equality of the power products 
in the resulting sets of 14 scalar equations. In nonsymmetrical 
situations (for example, the 3rd and 5th joints are prismatic 
and all the others are revolutes) it may be convenient to rear
range (4) by pre- and post-multiplying by the ,4,'s or their 
inverses in order to move matrices from one side to the other. 
Of course if we change (4), the systems p and I are measured 
in, and what they actually physically measure changes ac
cordingly. 
Finally, it is important to notice that a joint angle can always 
be eliminated from the equations for p and I by simply rear
ranging terms so that the corresponding Aj matrix appears in 
inverse form at the right-hand most position, on either side; 
as A6 does in (4). 
Prismatic Joints 
The foregoing method can be used for problem formulation 
and solution when determining the inverse kinematics of a 
series-chain manipulator which has one or more prismatic 
joints. This is because the basic mathematical structure of the 
14 scalar equations remains the same regardless of whether a 
joint is revolute or prismatic. If the fth joint is a revolute joint 
then we have c,- and s, as the variables which form the power 
products, while if it is prismatic we have d-, and dj instead. If 
we simply regroup the terms so that dt and dj rather than c,-
and si are treated as the variables, we obtain exactly the same 
number of power products. This means that the structure of 
our 14 equations is the same if we have revolute or prismatic 
joints. It is for this reason that changing one of the revolutes 
in the 6R to a prismatic joint does not alter the degree of the 
basic characteristic polynomial, it remains at 16. 
However, the degree of the characteristic polynomial is re
duced if we have two or more prismatic joints in a six-degree-
of-freedom manipulator. The reason for this is clear if we note 
that the d: variables cannot appear in the three components 
of the I equation. (Since these equations are from the rotation 
part of the matrix they must be independent of the joint trans
lations.) Moreover, each d, appears as a linear variable in the 
p equations and no products such as djdj are possible in p [by 
virtue of the structure of (1)]. It therefore follows that: 
(a) I does not contain any of the d,, 
(b) p, p • T and p x 1 contain only the d: and not any dj, 
(c) p • p and (p • p)T - (2p • T)p generally contain both dj 
and dj terms, and can contain djdj terms [if the two prismatic-
joint-matrices are both on the same side of the equal sign in 
(4)] but cannot contain any djdj or djdj terms. 
Thus if we have two prismatic joints we have less power 
products than in the 6R case, and this leads to a characteristic 
polynomial of lower degree. In Raghavan and Roth (1990) we 
show that when there are two prismatic joints and four revolute 
joints, the manipulator's characteristic polynomial is of degree 
eight and not sixteen. This is in agreement with the results in 
Duffy (1980), where most of these cases were first treated in 
a systematic manner. In Appendix 1 we briefly outline a pro
cedure which yields the 8th degree characteristic polynomial 
for the RPRRPR manipulator. This analysis serves to illustrate 
one way in which the number of power products can diminish 
as we add a second prismatic joints. 
For three prismatic joints the analysis becomes very simple 
since the T equations contain only the angular-displacement 
parameters. It always turns out that the 1 equation can be 
written so that one of its three components is a linear equation 
in the sine and/or cosine of one joint angle. This then can be 
rewritten as a second degree polynomial in the tangent of the 
half-angle, and it becomes the characteristic polynomial. Once 
the two roots of the characteristic polynomial are determined 
all the other variables follow linearly from the remaining two 
components of T, from the three scalar components of p, and 
from (4) as usual, Raghavan and Roth (1990). 
Characteristic Polynomials of Lower Degree 
We have seen that generally 6R and 5R,P manipulators have 
16-degree characteristic polynomials, 4R.2P manipulators have 
8-degree characteristic polynomials, and 3R,3P manipulators 
have characteristic polynomials of degree 2. (We have used a 
comma to signify that the prismatic joints can be located any 
place in the chain.) However, if a manipulator has special 
geometry, it is possible for its characteristic polynomial to be 
of lower degree than for the same joint types and number 
under a general geometry. This loss of degree is manifested in 
our solution method in three possible ways: a lowered number 
of power products in the initial equations for T and p, a lowered 
number of power products during the elimination phase, and/ 
or the coefficients of the highest order terms of the charac
teristic polynomial become zero. 
Furthermore, it is also possible under special geometry to 
lose constraint equations such that some of the joint variables, 
which are generally single-valued functions, become instead 
double-valued for each real root of the characteristic poly
nomial in the suppressed variable. 
We illustrate these effects by considering the case of a 6R 
manipulator with a wrist, i.e., the last three joint axes always 
intersect in a common point. If we substitute the special ge
ometry conditions for this case, i.e., a4 = a5 = 0, d$ = 0, we 
find that p becomes a function of only $lt 02 and 63. In terms 
of power products the third component of p yields 
(A')={B') r) , (16) 
where A' is a 1 x 1 matrix with entries that are linear com
binations of s3, c3, 1, and B' is a 1 x 2 matrix with entries 
that are constants. Furthermore, p • p yields 
(M')=(N') (!') , (17) 
506 / Vol. 115, SEPTEMBER 1993 Transactions of the ASME 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 6

- extraction_method: `direct`
- char_count: `6880`

```text
where M' is a 1 x 1 matrix with entries that are linear com
binations of S3, c3, 1, and N' is a 1 x 2 matrix with entries 
that are constants. If we substitute the tangent of the half-
angle formulas for Si and Ci we obtain, after clearing the 
denominators 
(A") \xA =0, and (M") I*, =0, (18) 
ever if we take the P and a neighboring R axis as coincident, 
the same analysis also includes the spatial RCRCR 5-bar and 
the RRCCR 5-bar as well as the RRCRC and RCRRC 5-bars, 
and the RCRRPR, CRRPR, RPRCR and RPRRC 6-bars. The 
same computer program which determines the RPRRPR char
acteristic polynomial and manipulator analysis, immediately 
gives the linkage analysis for all of these linkages. Analogous 
results follow from all the manipulators discussed in this paper. 
respectively. Where xt = tan((V2) and A " and M" are 1 x 
3 matrices with entries that contain linear combinations of s3, 
cs, 1. 
Using the dialytic elimination technique, we multiply each 
of these equations by xit and thereby obtain two additional 
equations. Now we have a set of four equations which can be 
written: 
0 
A" 
M" 
= 0. (19) 
/ 
Setting the determinant of the coefficient matrix to zero and 
substituting the tangent of the half-angle functions for s3 and 
C3, and then clearing the denominator by multiplying by (1 + 
X3)2, yields a characteristic polynomial of (only) degree 4 in 
X}. 
Generally, for each real root of x3, a unique value of 6^ 
follows from (16) and (17), and a unique value of 62 follows 
from the other two components of p. However, if either ax = 
0 or o?i = 0 then N' or B' are respectively zero, and we have 
to determine di from, respectively, either (16) or (17). This 
means we get two values of 81 for each root of x3. It turns out 
that in this case the characteristic polynomial degenerates to 
a quadratic, and we have at most two values for 03 at each 
hand position. 
In order to obtain 04 and 05 we use I. Since we know 6U 62, 
and 03, we can write T in the form 
= 0, (20) 
where A'" is a 3 x 3 matrix with elements which are linear 
functions of s5, c$, 1. It follows that we require that Det(^4'") 
= 0. It can be shown, see Appendix 2, that this yields a 
quadratic polynomial in x5. Once the values of 05 are obtained 
from this polynomial, we obtain a unique value of 64 (corre
sponding to each set of 0\, 02, 63 and 04) from (20). Finally 06 
follows as usual from (4). 
Conclusions 
We have presented a method for obtaining a characteristic 
polynomial for any series-manipulator with six joints which 
are combinations of revolute or prismatic joints. By virtue of 
the analogy to the closed-loop linkage analysis problem, this 
method will also determine the motion variables for closed-
loop spatial mechanism. In spatial mechanisms one finds ad
ditional types of joints, other than simply revolute and pris-, 
matic. However by virtue of the fact that any lower pair joint 
can be modeled as a combination of revolute and prismatic 
joints, it is clear that this analysis can be applied fairly broadly. 
Interestingly, joints such as cylindric joints and spherical 
joints represent special geometries, and for these the charac
teristic polynomials tend to reduce in degree. So for example 
the RPRRPR manipulator analysis presented in Appendix 1, 
also represents the analysis of a spatial 7-bar RRPRRPR. How-
Acknowledgments 
The financial support of the National Science Foundation 
is acknowledged. The computer program for the material in 
Appendix 1 was written by Mr. Konstantinos Mavroidis and 
financially supported by the The Robotics Laboratory of Paris. 
References 
Albala, H., and Angeles, J., 1979, "Numerical Solution to the Input-Output 
Displacement Equation of the General 7R Spatial Mechanism," Proceedings of 
the Fifth World Congress on Theory of Machines and Mechanisms, pp. 1008-
1011. 
Craig, J. J., 1989, Introduction to Robotics, 2nd edition, Addison Wesley, 
Reading, MA. 
Duffy, J., 1980, Analysis of Mechanisms and Manipulators, Wiley, New York. 
Duffy, J., and Crane, C, 1980, "A Displacement Analysis of the General 
Spatial 7R Mechanism," Mechanisms and Machine Theory, Vol. 15, pp. 153-
169. 
Lee, H-Y., and Liang, C-G., 1988a, "A New Vector Theory for the Analysis 
of Spatial Mechanisms," Mechanisms and Machine Theory, Vol. 23, No. 3, pp. 
209-217. 
Lee, H-Y., and Liang, C-G., 1988b, "Displacement Analysis of General Spa
tial 7-Link 7R Mechanism," Ibid, pp. 219-226. 
Paul, R. P., 1981, Robot Manipulators, MIT Press, Cambridge, MA. 
Pieper, D., 1968, The Kinematics of Manipulators Under Computer Control, 
Ph.D. Thesis, Stanford University. 
Pieper, D. L., and Roth, B., 1969, "The Kinematics of Manipulators Under 
Computer Control," Proceedings of the 2nd International Congress for the 
Theory of Machines and Mechanisms, Zakopane, Poland, Vol. 2, pp. 159-168. 
Raghavan, M., and Roth, B., 1989, "Kinematic Analysis of the 6R Manip
ulator of General Geometry," Proceedings of the 5th International Symposium 
on Robotics Research, H. Miura and S. Arimoto, eds., MIT Press, preprint 
1989, final 1990. 
Raghavan, M., 1990, "Manipulator Kinematics," Proceedings of the AMS 
Symposium on Mathematical Questions in robotics, R. Brockett, ed., Louisville, 
KY, PSAM Vol. 41, American Mathematical Society, Providence. 
Raghavan, M., and Roth, B., 1990, "A General Solution for the Inverse 
Kinematics of all Series Chains," Proceedings of the Eighth CISM-IFTOMM 
Symposium on Robots and Manipulators (ROMANSY-90), Cracow, Poland. 
Roth,B.,Rastegar, J.,andScheinman, V., 1973, "On the Design of Computer 
Controlled Manipulators," On the Theory and Practice of Robots and Manip
ulators, Vol. 1, First CISM-IFToMM Symposium, September, pp. 93-113. 
Salmon, G., 1885, Lessons Introductory to the Modern Higher Algebra, Chel
sea Publishing Co., New York. 
Tsai, L-W., and Morgan, A., 1989, "Solving the Kinematics of the Most 
General Six- and Five-Degree-of-Freedom Manipulators by Continuation Meth
ods," ASME JOURNAL OF MECHANISMS, TRANSMISSIONS, AND AUTOMATION IN 
DESIGN, Vol. 107, June, pp. 189-200. 
Yang, A. T., and Freudenstein, F., 1964, "Application of Dual-Number 
Quaternion Algebra to the Analysis of Spatial Mechanisms," ASME Journal 
of Applied Mechanics, Vol. 86, pp. 300-308. 
APPENDIX 1 
Characteristic Polynomial for the RPRRPR Manipu
lator 
For this case the d\ and d\ terms do not appear in_the power 
products of the 10 scalar equations obtained from 1, p, p • 1, 
p x I. Writing these 10 equations in matrix form yields a 
system of the following type: 
Journal of Mechanical Design SEPTEMBER 1993, Vol. 115 / 507 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

## Page 7

- extraction_method: `direct`
- char_count: `3842`

```text
- o 
0 
0 
S3 
(C3,l) 
(C3>1) 
1 
S} 
(CJ.1) 
-(C3.1) 
0 
0 
0 
C3 
•S3 
S3 
0 
(•S3,c3) 
(•S3.C3,1) 
(s3>c3,l) 
0 
0 
0 
*3 
(c3,l) 
(C3.1) 
0 
(s3,c3) 
(•S3.C3.1) 
(53,C3,1) 
(J3.C3) 
(J3.C3.1) 
(*3.C3,1) 
(•S3.C3) 
(S3,C3,l), 
(S3,C3,l) 
. 1 
(53.^3) 
(53,C3,1) 
(S3,C3,1) 
(•S3.C3) 
(S3,C3,1) 
(S3.C3.1) 
(•S3.C3) 
(•^3^3,1) 
(J3.C3.-I) 
1 
(J3.C3) 
(*3.C3,1) 
(•S3.C3,1) 
(*3,1) 
(C3.1) 
(C3,l) 
(^3.^3,1) 
(*3.C3,1) 
{jr3,c3,l) 
1 
(J3.C3.1) 
(53.^3,1) 
(•S3.C3.1) -
d5s4 
d5c4 
s4 
c4 
1 ?/ 
0 ) 
0 
0 
0 
0 
1 
1 
1 
0., 
IsA 
d2s\ 
d2ci 
\dj 
(£) = 0. 
In the (/J)th element (s3, c3, 1) represents X1//53 
+ .K2//C3 + K3jj, whereKiij, K2ij, K3jj, are constants that depend 
upon the manipulator's structural parameters. All terms with 
1 simply imply that A"uy = K%\j = 0 for that element. 
If we use the first two rows we can solve for S\ and c\. With 
these results the sixth row can be used to determine d2. Finally 
the seventh and eighth rows can now be used to determine 
d2cx. 
Substituting these results into rows 3, 4, 5,9, and 10 yields 
only 5 equations in the 6 power products (d5, d5s4, dsc4, s4, 
c4, 1). However a sixth equation can be obtained if we multiply 
the equation from the third row by d5. By virtue of the fact 
that this row did not originally have any ds terms, the resulting 
power products for this sixth equation are only (dss4, d5c4, 
d5). We now have a 6 x 6 system of the following form 
' d, " 
dss4 
d5c4 
s4 
c4 
Here E is a 6 x 6 matrix with entries which are linear in s3, 
c3, 1. Introducing the tangent of the half-angle substitution 
for S3 and c3, and then clearing the denominators yields 
rd5 ~ 
d5s4 
dsc4 
s4 
c4 
1 
The characteristic polynomial follows from IE' I =0. After 
removing the factor (1 + xf)2, this determinental equation 
yields a polynomial of degree 8 in x3. For each real root of 
the characteristic polynomial, the other joint variables follow 
in the usual manner from the linear systems developed during 
the elimination process. 
APPENDIX 2 
Determining Angles 04 and 05 from T when 0U 02, and 03 
Are Known 
From our_ equation for T, (7), it is clear that everything is 
known but r. Thus we can easily determine numerical values 
for r. Now with r known we can turn to the definition of r: 
(E') = 0. 
rx - c4rrii + s4m2 
r-i = - A3 (s4m{ - c4m2) + \i3m3 
(21) 
(22) 
r3 = ix3(s4mi 
This can be rewritten as 
- c4m2)+\3m3 (23) 
Setting the determinant of the coefficient matrix to zero yields, 
(wii + m\) (m3- fi3r2-\3r3) =0. 
Since {m\ + m\) ^ 0 we have 
Substituting from the definition of m3 yields 
- Csuws + (X4X5 - ix3r2 - \3r3) = 0, 
from which two values of 05 follow for each set of r2, r3. For 
each 05 one value of d4 follows from (21)-(23). 
Numerical Example 
For an RPRRPR manipulator with parameters: 
«i=1.46 ai = 135(deg.) ^=0.21 
When 
a2 = 0.56 a2 = 78 
a3 = 0.38 a3 = 23 
a4 = 0.56 a4 = 46 
as = 1.08 a5 = 35 
«6 = 0.67 a6 = 47 
/-0.4435 -0.6171 
[-0.1837 0.7724 
"1-0.8773 0.1502 
\ 0 0 
02 = 65 (deg.) 
d3 = 0.29 
rf4=-.54 
05 = 54 
rf6 = 0.48 
0.6500 -1.443 
0.6080 0.4665 
-0.4559 -2.0579 
0 1 / 
•A hand - 
The 8th degree characteristic polynomial is: 
1.077x1-3.547^ + 5.4954-16.37^ + 14.724 
-21.934+16.684-9.217x3 + 6.500 = 0 
This polynomial has two real roots: 
x} = 0.2979 and x3 = 0.805 
Using these roots yields, respectively, the following two sets 
of joint variables: 
01 = 165.0, d2 = 0.170, 03 = 77.7, 04 = 42.O, 
tf5=-1.08, 06=-9.OO; 
01 = 181.2,^2 = 0.340,03=142.9, 04=-21.5, 
ds= -0.264, 06=12.9 
508 / Vol. 115, SEPTEMBER 1993 Transactions of the ASME 
Downloaded from http://asmedigitalcollection.asme.org/mechanicaldesign/article-pdf/115/3/502/5920231/502_1.pdf?casa_token=Bi2nouGf3M8AAAAA:DtpqDQZM_WpaTr3I0AxO9aw0VMnse8rUDrbRJGjHA40EaluRNZPo8GPntkV_4Dd5A9v49tk by Ohio State University | OSU user on 14 February 2026
```

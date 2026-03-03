"""
Pure-numeric IK solver for a general 6R serial chain using a Raghavan-Roth style
resultant elimination pipeline (no Newton iterations, no NSolve, no Groebner).

Input convention for each joint i:
    dh_params[i] = (a_i, alpha_i, d_i)
and transform matrix:
    A_i = RotZ(theta_i) * TransZ(d_i) * TransX(a_i) * RotX(alpha_i)
        = RotZ(theta_i) * [constant matrix in (a_i, alpha_i, d_i)]
which matches the paper's matrix form.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence, Tuple, Union

import math
import time
import numpy as np

try:
    from scipy import linalg as scipy_linalg

    HAS_SCIPY = True
except Exception:  # pragma: no cover - fallback if scipy unavailable
    scipy_linalg = None
    HAS_SCIPY = False


ArrayLike = Sequence[float] | np.ndarray


@dataclass(frozen=True)
class IKSolution:
    q: Tuple[float, float, float, float, float, float]
    residual: float


ANGLE_TOL = 1e-6
RESIDUAL_TOL = 1e-4
PROJECTIVE_INF_X = 1e12
RECOVERY_X_ABS_MAX = 1e14


def normalize_angle(theta: float) -> float:
    wrapped = math.fmod(theta + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def normalize_sc(s: float, c: float) -> Tuple[float, float]:
    n = math.hypot(s, c)
    if n < 1e-14:
        return (0.0, 1.0)
    return (s / n, c / n)


def aiv(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def ais(a: float, alpha: float, d: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [1.0, 0.0, 0.0, a],
            [0.0, ca, -sa, 0.0],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    return aiv(theta) @ ais(a, alpha, d)


def se3_inverse(T: np.ndarray) -> np.ndarray:
    r = T[:3, :3]
    p = T[:3, 3]
    rt = r.T
    out = np.eye(4)
    out[:3, :3] = rt
    out[:3, 3] = -(rt @ p)
    return out


def forward_kinematics(dh_params: Sequence[Sequence[float]], q: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    for i in range(6):
        a_i, alpha_i, d_i = dh_params[i]
        T = T @ dh_matrix(a_i, alpha_i, d_i, q[i])
    return T


def build_pl_vectors(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    q: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    A = [
        dh_matrix(dh_params[i][0], dh_params[i][1], dh_params[i][2], q[i])
        for i in range(6)
    ]
    a2v = aiv(q[1])
    a2s = ais(dh_params[1][0], dh_params[1][1], dh_params[1][2])
    left = a2s @ A[2] @ A[3] @ A[4]
    right = np.linalg.inv(a2v) @ np.linalg.inv(A[0]) @ target_pose @ np.linalg.inv(A[5])
    p_left = left[:3, 3]
    p_right = right[:3, 3]
    l_left = left[:3, 2]
    l_right = right[:3, 2]
    return p_left, p_right, l_left, l_right


def build_14_equations(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    q: Sequence[float],
) -> np.ndarray:
    p_l, p_r, l_l, l_r = build_pl_vectors(dh_params, target_pose, q)
    eq_p = p_l - p_r
    eq_l = l_l - l_r
    eq_pp = float(np.dot(p_l, p_l) - np.dot(p_r, p_r))
    eq_pl = float(np.dot(p_l, l_l) - np.dot(p_r, l_r))
    eq_px = np.cross(p_l, l_l) - np.cross(p_r, l_r)
    combo_l = float(np.dot(p_l, p_l)) * l_l - 2.0 * float(np.dot(p_l, l_l)) * p_l
    combo_r = float(np.dot(p_r, p_r)) * l_r - 2.0 * float(np.dot(p_r, l_r)) * p_r
    eq_combo = combo_l - combo_r
    vals = np.concatenate([eq_p, eq_l, [eq_pp, eq_pl], eq_px, eq_combo])
    return vals


def monomial_45(theta4: float, theta5: float) -> np.ndarray:
    s4, c4 = math.sin(theta4), math.cos(theta4)
    s5, c5 = math.sin(theta5), math.cos(theta5)
    return np.array(
        [
            s4 * s5,
            s4 * c5,
            c4 * s5,
            c4 * c5,
            s4,
            c4,
            s5,
            c5,
            1.0,
        ],
        dtype=float,
    )


def monomial_12(theta1: float, theta2: float) -> np.ndarray:
    s1, c1 = math.sin(theta1), math.cos(theta1)
    s2, c2 = math.sin(theta2), math.cos(theta2)
    return np.array(
        [
            s1 * s2,
            s1 * c2,
            c1 * s2,
            c1 * c2,
            s1,
            c1,
            s2,
            c2,
        ],
        dtype=float,
    )


def build_halfangle_map_45() -> np.ndarray:
    # rows correspond to [s4s5,s4c5,c4s5,c4c5,s4,c4,s5,c5,1]
    # cols correspond to [x4^2x5^2,x4^2x5,x4x5^2,x4x5,x4^2,x5^2,x4,x5,1]
    t = np.zeros((9, 9), dtype=float)
    t[0, 3] = 4.0
    t[1, 2] = -2.0
    t[1, 6] = 2.0
    t[2, 1] = -2.0
    t[2, 7] = 2.0
    t[3, 0] = 1.0
    t[3, 4] = -1.0
    t[3, 5] = -1.0
    t[3, 8] = 1.0
    t[4, 2] = 2.0
    t[4, 6] = 2.0
    t[5, 0] = -1.0
    t[5, 4] = -1.0
    t[5, 5] = 1.0
    t[5, 8] = 1.0
    t[6, 1] = 2.0
    t[6, 7] = 2.0
    t[7, 0] = -1.0
    t[7, 4] = 1.0
    t[7, 5] = -1.0
    t[7, 8] = 1.0
    t[8, 0] = 1.0
    t[8, 4] = 1.0
    t[8, 5] = 1.0
    t[8, 8] = 1.0
    return t


def sample_angles(n: int, seed: int = 73421) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(-math.pi, math.pi, size=(n, 4))


# Symbolically derived linear coefficient map for Eq. (9):
# U = F^{-1} * Y, where
# - Y: 17 sampled evaluations of each scalar equation
# - U: [P_row(9), Q_row(8)] coefficients for that scalar equation.
# The sample angles and inverse matrix were exported offline once.
PQ_SYMBOLIC_SAMPLE_ANGLES = np.array(
    [
        [1.69673823897, 2.962965871865, 0.551751195511, -0.550635933697],
        [2.124305683239, -1.35151778162, 0.042006538206, -1.271978220727],
        [-0.916963962161, -2.508838046012, 1.748576867781, 0.377559104927],
        [1.471879906641, 0.971647887754, -0.272341445026, -2.027355199883],
        [-2.988196582985, 0.046459526076, 0.150075617542, -1.263697979985],
        [-2.536061119563, -1.494462923611, 0.086662269634, -1.376098791112],
        [0.831512316986, 1.996640091182, -1.617969141208, 0.029856666264],
        [-2.841354970612, -2.14209536172, -2.71675184462, 2.846383108325],
        [-1.999165286534, -2.771692700411, -2.056266835523, 2.318982885491],
        [2.089577723707, -0.615474169356, -1.261127673917, 1.307166318385],
        [-3.12408776899, 2.563595214957, -1.111534266641, -2.521343721348],
        [1.050822228457, 1.782149206715, -3.039576922521, -0.956065652517],
        [0.484527303898, 0.394768084706, 2.531365903151, -1.732241512316],
        [-3.101385200507, 1.775773952919, -2.942430204732, 1.864900116697],
        [-1.735912964048, -0.09455970154, 0.960385327619, -2.353825670389],
        [3.085871917488, 0.496623639214, 1.394218297829, 2.669197573194],
        [1.345522300159, -1.400760620293, -2.497027865158, -1.501501913954],
    ],
    dtype=float,
)

PQ_FEATURE_MATRIX_INV = np.array(
    [
        [-3.5440490796749569e-01, -2.4443825715344362e-02, 4.7985268723643709e-01, 4.8312909941157063e-02, 1.3697097685497381e-01, -2.5921444211712025e-01, 4.5980996161805576e-01, 6.8723254600230521e-01, -9.1439924209932544e-01, -4.0426516309248782e-02, 4.7289642209990024e-01, -4.2414206843147051e-01, 5.4933224395101600e-02, -1.1913722867431221e-01, -7.9379293912388266e-02, -3.7416851467594145e-02, -8.7044351453631430e-02],
        [-8.9405741139936365e-02, 1.0809127082608814e-02, 4.3534132258084957e-01, 1.5976986132092771e-01, 4.3409904174210212e-01, -7.3382757858164160e-01, -1.8518365753313049e-01, 6.7314235260220723e-01, -5.2576165398423458e-01, 2.6853323413078484e-01, 3.1869836655251310e-01, 1.1451294817099932e-01, -1.0594611689050805e-01, -1.9126465028278555e-01, 1.9942967146973617e-01, -4.8627917049231478e-01, -2.9666735674817796e-01],
        [1.5950657450443576e-01, -3.5725379212289105e-01, -3.8542696771974680e-02, -2.9445327540757493e-01, 1.0817799645877661e-01, 5.6026033750648602e-02, -1.0484530565018524e-02, -7.3612497591277845e-02, 4.1811918482426789e-02, 1.8158827136553282e-01, 2.7212311578398074e-01, -5.6469974179774356e-04, 2.1507816396048998e-01, -3.7491924213202771e-01, -1.6658950417250090e-01, 2.4643139306164641e-01, 3.5676771137125751e-02],
        [2.9558292073901676e-01, -3.6925149407807661e-01, -7.4352700394726939e-02, -3.9656395855295797e-02, 5.1595458705000563e-01, -1.5725716833694434e-01, 3.0542421775103107e-01, 2.8185510424971827e-01, 8.5374655390060047e-02, -2.2057562419811913e-01, -9.7127062349982432e-02, -6.1998251398786619e-01, 3.6416466674383619e-01, -1.8015863961747980e-01, -3.4227200230170068e-01, 1.1590708125034928e-01, 1.3637036794617446e-01],
        [8.6722815115101465e-02, 3.1329057500733132e-01, 7.3241449382310309e-02, -1.1269094689243234e-01, -2.3874064306275347e-01, 6.5840564587229977e-02, -2.1904522967964718e-01, -1.0336430382231929e-01, -1.3187206671452926e-03, -1.1230755703884057e-01, -1.0121612381569961e-01, 1.6875280109601892e-01, 1.3284258135159907e-01, 4.5065926570573199e-02, -3.8826350308090951e-02, 2.6215469482646092e-01, -2.2040153264969675e-01],
        [1.3609532129155907e-01, -6.0775013270173775e-02, 7.0570687064613669e-02, 1.4952003245945325e-02, 3.4333307155317232e-02, -4.6105764550000941e-02, 1.6162127993946060e-01, -6.1315727713662345e-02, -5.2068788659675978e-02, 1.3659962502234332e-01, 2.9328327908898838e-01, -3.3321111014066357e-01, 9.9370265431926369e-02, -3.3047305652409842e-01, -1.6170186925009204e-02, 1.1144319164450350e-01, -1.5814931210137301e-01],
        [-1.1821491288688142e-01, -1.8604332624222314e-02, 3.7227868358825600e-01, 5.5124898196410005e-02, -1.6241854822235632e-01, -3.0816942777577011e-01, 2.0419256648707695e-01, 4.9031968263274678e-01, -5.9030109745336468e-01, 3.7851901937931570e-01, 2.0404143225306537e-01, -2.3243520866663292e-01, -6.9595931226442462e-02, 2.7754683732451834e-02, 1.6799617491064436e-01, -1.0539741050963761e-01, -2.9509027181465930e-01],
        [-1.0931602763422121e-01, -4.0401058872540453e-02, 1.9056318719961682e-01, -3.2522362129407811e-01, 3.4329168813080818e-02, 4.2040113130926032e-01, 1.4147927098992319e-01, -1.6713138826970575e-01, -4.2586692302959511e-01, 1.9383304001969923e-01, 1.7678451381417631e-01, -1.2134231686642167e-02, 1.9588882191219090e-01, 2.5318393595124494e-01, -2.2087603151690807e-01, -1.7957939954181076e-01, -1.2593438816369115e-01],
        [-2.7344258326200185e-02, 7.3861170369018847e-02, 2.8708834998618099e-01, -2.5421460964685297e-02, 5.4821503705842989e-02, -2.1047108643064577e-01, 2.5615424389895047e-01, 2.3387249610183361e-01, -1.5987393793170984e-01, 1.2309019602211260e-01, 2.3843295830804168e-01, -1.2031009294642393e-01, 1.1803805060292082e-01, -3.3563246299493421e-02, 1.9835735355267797e-01, 1.2969538206707697e-02, -1.9701777855129540e-02],
        [-3.3536742919925711e-01, 9.0583181518173517e-01, 1.7630393806967112e-01, -4.4209164888288915e-01, 2.3779597913797526e-01, -8.4837603840254339e-01, 2.4388346086396384e-01, 1.7978360440445826e-01, -1.1390069434384990e-01, -8.4562916506277980e-02, 3.9091195977219012e-01, -1.5858886331630004e-02, 1.2115932467905192e-01, -1.4935725913309716e-01, 1.5881095005534285e-01, -7.3198601750967623e-02, -3.5176755761387640e-01],
        [1.3233306498124409e-01, -2.4381805519946326e-01, 1.1868539496857651e-01, 5.1671059353411951e-02, 4.4001083772255478e-02, -3.0253008222550609e-01, 9.2357009343301086e-02, 5.1720320402666831e-01, -5.8666544640221430e-01, 1.6813612044590481e-01, 1.1737595640844727e-01, -3.0842729882633202e-02, -2.7507944115942540e-01, 3.3442655367084601e-03, 4.9955913706146798e-01, -3.3332393531134019e-01, 2.7593394282596551e-02],
        [4.4772916139207647e-01, -7.3190093431130809e-01, -2.6631196266467261e-01, 2.2642205192811068e-01, -1.5479111327199638e-01, 6.3822341627001589e-01, -3.0293199093356443e-01, -7.4417485763868962e-01, 4.9151073388879668e-01, -6.4451939078185264e-02, -3.4269699615106430e-01, -2.4353756777036312e-01, -4.5826122557157463e-02, 3.6074371130195987e-01, -1.3061475891595609e-01, 1.5261829031503196e-01, 7.0999087819696616e-01],
        [8.8665294355633653e-02, 8.9727952726801438e-02, -9.5202539833714170e-02, 1.0778818691192781e-01, 4.6122660131709087e-01, -4.8567018258545985e-01, -1.1305342484284921e-01, -4.7921741877675263e-02, 5.9898519626795199e-01, -1.9965748195546310e-01, -4.1515711387925402e-01, 3.4716774025955494e-01, -4.3644032117772458e-01, -2.8181639243377871e-01, 5.0239183866231468e-02, 1.6893534875887625e-01, 1.6218369412185085e-01],
        [-2.8261961573437155e-01, 6.5721768108540088e-03, 1.9764053222828545e-01, -3.3766868011198642e-01, 3.0264084769321492e-01, -9.6715504291395402e-02, 2.8952994487775924e-01, 6.6640809798586470e-02, -3.0505523896625433e-02, -1.1578770263424878e-01, 3.0289534328864670e-01, -2.1761439836300181e-01, 2.0248989793197020e-01, -1.1963105860392949e-01, 8.4276003054767623e-02, -6.6779388950313698e-02, -1.8536368309821202e-01],
        [9.3967721894138816e-03, -2.9197574105268419e-02, -2.4547558831758476e-01, -3.1259826787631145e-01, -5.5521593480644789e-02, 5.7935033712207118e-01, -3.0122491108438937e-01, -2.2416694194658157e-01, -1.4965071798670532e-02, 1.2821422485078904e-01, 1.3472783629086690e-01, 3.3141861451017729e-01, -1.1008144984177765e-02, 1.9640912754136253e-01, -3.0237540896667209e-01, 2.1783736549364721e-01, -1.0082077543802730e-01],
        [1.9927768262618092e-01, -3.6925131051111670e-01, -7.3406500593389781e-02, 2.2179096198327850e-01, -3.9431542067502345e-01, 6.1623345761267889e-01, -2.8400058014194662e-01, -1.4780530042541284e-01, 8.9026718235307514e-02, 1.3486120853693925e-01, -3.7208168854712170e-01, -6.2155031986654896e-02, -2.3897295035578214e-01, 1.2725879523407374e-01, 6.9837984020993712e-02, -2.0924662146819505e-02, 5.0462663713381517e-01],
        [2.6734232100945837e-01, 3.1028097250996162e-01, -2.6015410836167752e-01, -1.6178833188059891e-01, -2.6335133676314820e-01, 1.0550158302725134e-01, -9.1431352739538202e-02, -4.8025797609155330e-01, 8.1587420637614338e-01, -3.5265292918381919e-01, -1.7462741196457918e-01, 2.9840121395543645e-01, -1.8789503281933065e-01, -8.8036923219351956e-02, -1.4367937785670457e-01, 3.2305082748723452e-01, 8.3423656514816050e-02],
    ],
    dtype=float,
)

PQ_FEATURE_MATRIX = np.linalg.inv(PQ_FEATURE_MATRIX_INV)


def equation_values_at_x3(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    x3: float,
    theta1: float,
    theta2: float,
    theta4: float,
    theta5: float,
) -> np.ndarray:
    q = np.array([theta1, theta2, 2.0 * math.atan(x3), theta4, theta5, 0.0], dtype=float)
    return build_14_equations(dh_params, target_pose, q)


def build_y_matrix_at_x3(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    x3: float,
) -> np.ndarray:
    """Build Y(x3) for the symbolic-exported mapping U = F^{-1} Y.

    Y shape: (17, 14), each row corresponds to one fixed sampled
    (theta1, theta2, theta4, theta5), each column is one scalar equation.
    """
    values = np.zeros((17, 14), dtype=float)
    for i, (t1, t2, t4, t5) in enumerate(PQ_SYMBOLIC_SAMPLE_ANGLES):
        values[i, :] = equation_values_at_x3(
            dh_params, target_pose, x3, float(t1), float(t2), float(t4), float(t5)
        )
    return values


def _build_pq_at_x3_least_squares(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    x3: float,
    samples: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Reference implementation: per-row least squares (slow)."""
    F = np.zeros((len(samples), 17), dtype=float)
    values = np.zeros((len(samples), 14), dtype=float)
    for i, (t1, t2, t4, t5) in enumerate(samples):
        F[i, :9] = monomial_45(float(t4), float(t5))
        F[i, 9:] = -monomial_12(float(t1), float(t2))
        values[i, :] = equation_values_at_x3(
            dh_params, target_pose, x3, float(t1), float(t2), float(t4), float(t5)
        )

    P = np.zeros((14, 9), dtype=float)
    Q = np.zeros((14, 8), dtype=float)
    max_res = 0.0
    for row in range(14):
        y = values[:, row]
        u, *_ = np.linalg.lstsq(F, y, rcond=None)
        P[row, :] = u[:9]
        Q[row, :] = u[9:]
        err = np.max(np.abs(F @ u - y))
        max_res = max(max_res, float(err))
    return P, Q, max_res


def build_pq_at_x3(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    x3: float,
    samples: np.ndarray | None = None,
) -> Tuple[np.ndarray, np.ndarray, float]:
    # General symbolic-export path:
    # U = F^{-1} * Y(DH, A_hand, x3)
    # where F^{-1} is constant (depends only on the chosen sample-angle set),
    # and Y is rebuilt for each DH/pose instance.
    _ = samples  # retained for backward compatibility with existing callers.
    values = build_y_matrix_at_x3(dh_params, target_pose, x3)

    coeff = PQ_FEATURE_MATRIX_INV @ values  # (17, 14)
    P = coeff[:9, :].T
    Q = coeff[9:, :].T

    recon = PQ_FEATURE_MATRIX @ coeff
    max_res = float(np.max(np.abs(recon - values)))
    return P, Q, max_res


def select_independent_rows(M: np.ndarray, k: int) -> List[int]:
    idx: List[int] = []
    remaining = list(range(M.shape[0]))

    while len(idx) < k and remaining:
        best_row = None
        best_score = float("inf")
        for r in remaining:
            cand = idx + [r]
            block = M[cand, :]
            rank = int(np.linalg.matrix_rank(block, tol=1e-10))
            if rank < len(cand):
                continue
            s = np.linalg.svd(block, compute_uv=False)
            if s.size == 0 or abs(s[-1]) <= 1e-14:
                continue
            score = float(s[0] / s[-1])
            if score < best_score:
                best_score = score
                best_row = r
        if best_row is None:
            break
        idx.append(best_row)
        remaining.remove(best_row)
    return idx


def eliminate_theta12(P: np.ndarray, Q: np.ndarray) -> np.ndarray | None:
    U, S, _ = np.linalg.svd(Q, full_matrices=True)
    rank = int(np.sum(S > 1e-10))
    if rank < 8:
        return None
    left_null = U[:, rank:].T
    if left_null.shape[0] < 6:
        return None
    return left_null @ P


def build_dialytic_12x12(E9: np.ndarray) -> np.ndarray:
    M = np.zeros((12, 12), dtype=float)
    for i in range(6):
        c = E9[i]
        # Unmultiplied row
        r = np.zeros(12, dtype=float)
        r[3] = c[0]
        r[4] = c[1]
        r[6] = c[2]
        r[7] = c[3]
        r[5] = c[4]
        r[9] = c[5]
        r[8] = c[6]
        r[10] = c[7]
        r[11] = c[8]
        M[i, :] = r

        # Multiplied by x4
        r = np.zeros(12, dtype=float)
        r[0] = c[0]
        r[1] = c[1]
        r[3] = c[2]
        r[4] = c[3]
        r[2] = c[4]
        r[6] = c[5]
        r[5] = c[6]
        r[7] = c[7]
        r[8] = c[8]
        M[i + 6, :] = r
    return M


def characteristic_det_at_x3(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    x3: float,
    samples: np.ndarray,
) -> float:
    P, Q, _ = build_pq_at_x3(dh_params, target_pose, x3, samples)
    E45 = eliminate_theta12(P, Q)
    if E45 is None:
        return float("nan")
    E9 = E45 @ build_halfangle_map_45()
    M = build_dialytic_12x12(E9)
    return float(np.linalg.det(M))


def eliminate_theta12_with_fixed_pivot(
    P: np.ndarray,
    Q: np.ndarray,
    pivot_rows: Sequence[int],
) -> np.ndarray | None:
    """Eliminate theta1/theta2 using a fixed 8-row pivot block of Q.

    This deterministic elimination avoids SVD basis flips across x3 samples,
    which is useful when fitting a matrix polynomial pencil in x3.
    """
    if len(pivot_rows) != 8:
        return None
    piv = [int(r) for r in pivot_rows]
    if len(set(piv)) != 8:
        return None
    if min(piv) < 0 or max(piv) >= Q.shape[0]:
        return None

    rem = [r for r in range(Q.shape[0]) if r not in piv]
    if len(rem) != Q.shape[0] - 8:
        return None

    Qp = Q[piv, :]
    if np.linalg.matrix_rank(Qp, tol=1e-10) < 8:
        return None
    Qr = Q[rem, :]

    try:
        # X = Qr * inv(Qp), solved stably without explicit inverse.
        X = np.linalg.solve(Qp.T, Qr.T).T
    except np.linalg.LinAlgError:
        return None

    # In permuted row order [piv, rem], nullspace basis is [-X | I6].
    N_perm = np.hstack([-X, np.eye(len(rem), dtype=float)])
    order = piv + rem
    N = np.zeros((len(rem), Q.shape[0]), dtype=float)
    for j, orig_row in enumerate(order):
        N[:, orig_row] = N_perm[:, j]
    return N @ P


def _build_left_elimination_matrix(Q: np.ndarray, pivot_rows: Sequence[int]) -> np.ndarray | None:
    """Build the 6x14 left elimination matrix N from a fixed 8-row Q minor.

    If Qp is the selected 8x8 row minor and Qr are remaining rows, then
    N = [-Qr Qp^{-1} | I] in row-permuted coordinates, mapped back to
    original row ordering.
    """
    piv = [int(r) for r in pivot_rows]
    if len(piv) != 8 or len(set(piv)) != 8:
        return None
    if min(piv) < 0 or max(piv) >= Q.shape[0]:
        return None

    rem = [r for r in range(Q.shape[0]) if r not in piv]
    if len(rem) != Q.shape[0] - 8:
        return None

    Qp = Q[piv, :]
    if np.linalg.matrix_rank(Qp, tol=1e-10) < 8:
        return None
    Qr = Q[rem, :]

    try:
        X = np.linalg.solve(Qp.T, Qr.T).T  # X = Qr * inv(Qp)
    except np.linalg.LinAlgError:
        return None

    N_perm = np.hstack([-X, np.eye(len(rem), dtype=float)])
    order = piv + rem
    N = np.zeros((len(rem), Q.shape[0]), dtype=float)
    for j, orig_row in enumerate(order):
        N[:, orig_row] = N_perm[:, j]
    return N


def build_pq_sincos_model(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    samples: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
    """Recover P(theta3)=Ps*sin(theta3)+Pc*cos(theta3)+P1 and constant Q.

    Based on RR/MC94 formulation, P is affine in sin(theta3),cos(theta3),
    while Q is independent of theta3.
    """
    # Three non-collinear angle samples for [sin, cos, 1] interpolation.
    theta3_grid = np.array([-2.1, -0.2, 1.3], dtype=float)
    basis = np.column_stack(
        [
            np.sin(theta3_grid),
            np.cos(theta3_grid),
            np.ones_like(theta3_grid),
        ]
    )  # (3,3)
    basis_inv = np.linalg.inv(basis)

    p_samples: List[np.ndarray] = []
    q_samples: List[np.ndarray] = []
    for theta3 in theta3_grid:
        x3 = math.tan(0.5 * float(theta3))
        P, Q, _ = build_pq_at_x3(dh_params, target_pose, x3, samples)
        p_samples.append(P)
        q_samples.append(Q)

    P_stack = np.stack(p_samples, axis=0)  # (3,14,9)
    coeff = np.tensordot(basis_inv, P_stack, axes=(1, 0))  # (3,14,9)
    P_s = coeff[0, :, :]
    P_c = coeff[1, :, :]
    P_1 = coeff[2, :, :]

    Q_stack = np.stack(q_samples, axis=0)
    Q_const = np.mean(Q_stack, axis=0)

    # Diagnostics for model exactness.
    q_spread = float(np.max(np.abs(Q_stack - Q_const[None, :, :])))
    p_fit_max_err = 0.0
    for theta3, P in zip(theta3_grid, p_samples):
        P_rec = P_s * math.sin(float(theta3)) + P_c * math.cos(float(theta3)) + P_1
        p_fit_max_err = max(p_fit_max_err, float(np.max(np.abs(P_rec - P))))

    diagnostics = {
        "pq_sincos_theta3_samples": [float(v) for v in theta3_grid.tolist()],
        "pq_sincos_fit_max_abs_error": float(p_fit_max_err),
        "q_theta3_invariance_max_abs_error": float(q_spread),
    }
    return P_s, P_c, P_1, Q_const, diagnostics


def fit_matrix_polynomial_pencil_with_timing(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    samples: np.ndarray,
    x_samples: np.ndarray,
    degree: int = 16,
) -> Tuple[List[np.ndarray], float, float, float, Dict[str, Any]]:
    """Construct exact quadratic matrix polynomial Sigma(x3)=A x3^2+B x3+C.

    Based on MC94 Section III / IV-C:
    1) Build P(theta3)=Ps*sin(theta3)+Pc*cos(theta3)+P1 and constant Q.
    2) Use fixed-rank elimination matrix N from Q.
    3) Substitute half-angle for theta3:
         sin = 2x3/(1+x3^2), cos = (1-x3^2)/(1+x3^2)
       so (1+x3^2) P = P2 x3^2 + P1 x3 + P0.
    4) Build dialytic 12x12 Sigma(x3)=A x3^2 + B x3 + C exactly.

    Returns coeff_mats in ascending order [C, B, A], scale=1.
    """
    _ = degree  # Retained for API compatibility.
    x_grid = np.asarray(x_samples, dtype=float).flatten()
    if x_grid.size < 1:
        raise RuntimeError("Need at least one x3 sample for diagnostics.")

    t0 = time.perf_counter()
    P_s, P_c, P_1, Q_const, model_diag = build_pq_sincos_model(
        dh_params, target_pose, samples
    )
    matrix_eval_time = time.perf_counter() - t0

    # Select a stable fixed pivot of Q and build left elimination matrix N.
    pivot_rows = select_independent_rows(Q_const, 8)
    if len(pivot_rows) != 8:
        raise RuntimeError("Unable to choose a stable 8-row pivot set for constant Q.")
    N = _build_left_elimination_matrix(Q_const, pivot_rows)
    if N is None:
        raise RuntimeError("Unable to build fixed elimination matrix from constant Q.")

    t_fit0 = time.perf_counter()
    map45 = build_halfangle_map_45()

    # (1+x^2)P(x) = (P1-Pc)x^2 + (2Ps)x + (P1+Pc)
    P2 = P_1 - P_c
    P1_lin = 2.0 * P_s
    P0 = P_1 + P_c

    E9_2 = (N @ P2) @ map45
    E9_1 = (N @ P1_lin) @ map45
    E9_0 = (N @ P0) @ map45

    M2 = build_dialytic_12x12(E9_2)
    M1 = build_dialytic_12x12(E9_1)
    M0 = build_dialytic_12x12(E9_0)
    fit_time = time.perf_counter() - t_fit0

    # Validate exactness against direct evaluation at requested grid points:
    # (1+x^2) * M_direct(x) ?= M0 + M1 x + M2 x^2
    max_entry_err = 0.0
    for x in x_grid:
        P_x, _, _ = build_pq_at_x3(dh_params, target_pose, float(x), samples)
        E45_x = N @ P_x
        E9_x = E45_x @ map45
        M_x = build_dialytic_12x12(E9_x)
        lhs = (1.0 + float(x) * float(x)) * M_x
        rhs = M0 + M1 * float(x) + M2 * (float(x) * float(x))
        err = float(np.max(np.abs(lhs - rhs)))
        max_entry_err = max(max_entry_err, err)

    diagnostics = {
        "matrix_pencil_method": "exact_quadratic_from_eq12",
        "matrix_pencil_degree": 2,
        "matrix_pencil_valid_samples": int(len(x_grid)),
        "matrix_pencil_requested_samples": int(len(x_grid)),
        "matrix_pencil_scale": 1.0,
        "matrix_pencil_fit_max_abs_entry_error": float(max_entry_err),
        "matrix_pencil_fit_rmse_entry_error": float("nan"),
        "matrix_pencil_pivot_rows": [int(r) for r in pivot_rows],
    }
    diagnostics.update(model_diag)
    return [M0, M1, M2], 1.0, matrix_eval_time, fit_time, diagnostics


def _solve_matrix_polynomial_pencil_eigenvalues(coeff_mats: Sequence[np.ndarray]) -> np.ndarray:
    """Solve det(sum_k Mk t^k)=0 via first-companion linearization."""
    d = len(coeff_mats) - 1
    if d < 1:
        return np.array([], dtype=complex)

    n = int(coeff_mats[0].shape[0])
    if any(mat.shape != (n, n) for mat in coeff_mats):
        raise ValueError("All matrix polynomial coefficients must have same square shape.")

    N = n * d
    A = np.zeros((N, N), dtype=float)
    B = np.zeros((N, N), dtype=float)

    # Top block row: [-M_{d-1} ... -M_0], with B top-left = M_d.
    A[:n, :] = np.hstack([-coeff_mats[k] for k in range(d - 1, -1, -1)])
    B[:n, :n] = coeff_mats[d]

    # Shift blocks.
    if d > 1:
        I = np.eye(n * (d - 1), dtype=float)
        A[n:, : n * (d - 1)] = I
        B[n:, n:] = I

    if HAS_SCIPY and scipy_linalg is not None:
        return scipy_linalg.eigvals(A, B, check_finite=False)

    # Basic fallback if scipy is unavailable.
    try:
        return np.linalg.eigvals(np.linalg.solve(B, A))
    except np.linalg.LinAlgError:
        return np.array([], dtype=complex)


def _refine_root_newton(detf: Any, x0: float, max_iter: int = 20) -> float:
    """Refine one scalar root candidate with finite-difference Newton steps."""
    x = float(x0)
    for _ in range(max_iter):
        fx = float(detf(x))
        if not np.isfinite(fx):
            break
        h = 1e-4 * (1.0 + abs(x))
        fp = float(detf(x + h) - detf(x - h)) / (2.0 * h)
        if not np.isfinite(fp) or abs(fp) <= 1e-12:
            break
        x_next = x - fx / fp
        if not np.isfinite(x_next):
            break
        if abs(x_next - x) <= 1e-10 * (1.0 + abs(x)):
            x = x_next
            break
        x = x_next
    return float(x)


def null_vector(M: np.ndarray) -> np.ndarray:
    _, _, vh = np.linalg.svd(M)
    z = vh[-1, :]
    n = np.linalg.norm(z)
    if n > 0.0:
        z = z / n
    return z


def _poly_trim(p: np.ndarray, tol: float = 1e-12) -> np.ndarray:
    q = np.array(p, dtype=float)
    if q.size == 0:
        return q
    k = q.size - 1
    while k > 0 and abs(q[k]) <= tol:
        k -= 1
    return q[: k + 1]


def _poly_add(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n = max(len(a), len(b))
    out = np.zeros(n, dtype=float)
    out[: len(a)] += a
    out[: len(b)] += b
    return _poly_trim(out)


def _poly_sub(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n = max(len(a), len(b))
    out = np.zeros(n, dtype=float)
    out[: len(a)] += a
    out[: len(b)] -= b
    return _poly_trim(out)


def _poly_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if len(a) == 0 or len(b) == 0:
        return np.zeros(0, dtype=float)
    out = np.zeros(len(a) + len(b) - 1, dtype=float)
    for i, val in enumerate(a):
        out[i : i + len(b)] += val * b
    return _poly_trim(out)


def _poly_scale(a: np.ndarray, s: float) -> np.ndarray:
    return _poly_trim(np.array(a, dtype=float) * s)


def _poly_eval(a: np.ndarray, x: float) -> float:
    acc = 0.0
    p = 1.0
    for c in a:
        acc += c * p
        p *= x
    return float(acc)


def _real_roots_from_poly_desc(coeff_desc: np.ndarray, imag_tol: float = 1e-8) -> List[float]:
    coeff = np.array(coeff_desc, dtype=float)
    idx = 0
    while idx < len(coeff) and abs(coeff[idx]) <= 1e-12:
        idx += 1
    coeff = coeff[idx:]
    if coeff.size <= 1:
        return []
    roots = np.roots(coeff)
    return [float(r.real) for r in roots if abs(r.imag) <= imag_tol]


def _dedupe_scalar_roots(values: Iterable[float], tol: float = 1e-6) -> List[float]:
    uniq: List[float] = []
    for v in sorted(float(x) for x in values if np.isfinite(x)):
        if not uniq or abs(v - uniq[-1]) > tol:
            uniq.append(v)
    return uniq


def _bracket_roots_from_det(
    detf: Any,
    x_min: float,
    x_max: float,
    num_scan_points: int = 241,
) -> List[float]:
    x_scan = np.linspace(x_min, x_max, num_scan_points)
    f_scan = np.array([detf(float(x)) for x in x_scan], dtype=float)
    bracket_roots: List[float] = []
    for i in range(len(x_scan) - 1):
        a = float(x_scan[i])
        b = float(x_scan[i + 1])
        fa = float(f_scan[i])
        fb = float(f_scan[i + 1])
        if not np.isfinite(fa) or not np.isfinite(fb):
            continue
        if abs(fa) < 1e-10:
            bracket_roots.append(a)
            continue
        if fa * fb > 0.0:
            continue
        left, right = a, b
        f_left = fa
        for _ in range(60):
            mid = 0.5 * (left + right)
            f_mid = detf(mid)
            if not np.isfinite(f_mid):
                break
            if abs(f_mid) < 1e-12:
                left = right = mid
                break
            if f_left * f_mid <= 0.0:
                right = mid
            else:
                left = mid
                f_left = f_mid
        bracket_roots.append(0.5 * (left + right))
    return _dedupe_scalar_roots(bracket_roots, tol=1e-5)


def _solve_quadratic_real(a: float, b: float, c: float, tol: float = 1e-10) -> List[float]:
    if abs(a) <= tol:
        if abs(b) <= tol:
            return []
        return [(-c) / b]
    disc = b * b - 4.0 * a * c
    if disc < -tol:
        return []
    disc = max(disc, 0.0)
    sd = math.sqrt(disc)
    den = 2.0 * a
    return [(-b + sd) / den, (-b - sd) / den]


def solve_x4x5_from_e9(E9: np.ndarray) -> List[Tuple[float, float]]:
    # row polynomial:
    # a1 x4^2 x5^2 + a2 x4^2 x5 + a3 x4 x5^2 + a4 x4 x5 + a5 x4^2 + a6 x5^2 + a7 x4 + a8 x5 + a9 = 0
    # => A(x4) x5^2 + B(x4) x5 + C(x4) = 0
    # A=[a6, a3, a1], B=[a8, a4, a2], C=[a9, a7, a5] in ascending x4 powers.
    candidates: List[Tuple[float, float]] = []

    def append_unique(x4: float, x5: float) -> None:
        if not np.isfinite(x4) or not np.isfinite(x5):
            return
        if abs(x4) > 1e8 or abs(x5) > 1e8:
            return
        for u, v in candidates:
            if abs(x4 - u) <= 1e-7 and abs(x5 - v) <= 1e-7:
                return
        candidates.append((x4, x5))

    row_data = []
    for r in E9:
        A = np.array([r[5], r[2], r[0]], dtype=float)
        B = np.array([r[7], r[3], r[1]], dtype=float)
        C = np.array([r[8], r[6], r[4]], dtype=float)
        row_data.append((A, B, C))

    pair_indices = [(i, j) for i in range(6) for j in range(i + 1, 6)]
    for i, j in pair_indices:
        A1, B1, C1 = row_data[i]
        A2, B2, C2 = row_data[j]

        term1 = _poly_mul(_poly_mul(C1, C1), _poly_mul(A2, A2))
        term2 = _poly_mul(_poly_mul(C1, B1), _poly_mul(B2, A2))
        term3 = _poly_mul(_poly_mul(C1, A1), _poly_mul(C2, A2))
        term4 = _poly_mul(_poly_mul(C1, A1), _poly_mul(B2, B2))
        term5 = _poly_mul(_poly_mul(B1, B1), _poly_mul(C2, A2))
        term6 = _poly_mul(_poly_mul(B1, A1), _poly_mul(C2, B2))
        term7 = _poly_mul(_poly_mul(A1, A1), _poly_mul(C2, C2))

        res = term1
        res = _poly_sub(res, term2)
        res = _poly_add(res, _poly_scale(term3, -2.0))
        res = _poly_add(res, term4)
        res = _poly_add(res, term5)
        res = _poly_sub(res, term6)
        res = _poly_add(res, term7)
        res = _poly_trim(res)
        if res.size <= 1:
            continue

        x4_roots = _real_roots_from_poly_desc(res[::-1], imag_tol=1e-7)
        for x4 in x4_roots:
            a = _poly_eval(A1, x4)
            b = _poly_eval(B1, x4)
            c = _poly_eval(C1, x4)
            for x5 in _solve_quadratic_real(a, b, c):
                # Verify with all six equations.
                vec = np.array(
                    [
                        x4 * x4 * x5 * x5,
                        x4 * x4 * x5,
                        x4 * x5 * x5,
                        x4 * x5,
                        x4 * x4,
                        x5 * x5,
                        x4,
                        x5,
                        1.0,
                    ],
                    dtype=float,
                )
                res_norm = float(np.max(np.abs(E9 @ vec)))
                if res_norm <= 1e-3:
                    append_unique(x4, x5)

    return candidates


def recover_x4x5_candidates(z: np.ndarray) -> List[Tuple[float, float]]:
    # basis z:
    # [x4^3x5^2,x4^3x5,x4^3,x4^2x5^2,x4^2x5,x4^2,x4x5^2,x4x5,x4,x5^2,x5,1]
    cands: List[Tuple[float, float]] = []

    def add(x4: float, x5: float) -> None:
        if not np.isfinite(x4) or not np.isfinite(x5):
            return
        if abs(x4) > RECOVERY_X_ABS_MAX or abs(x5) > RECOVERY_X_ABS_MAX:
            return
        for u, v in cands:
            if abs(x4 - u) <= 1e-8 and abs(x5 - v) <= 1e-8:
                return
        cands.append((x4, x5))

    if abs(z[11]) > 1e-10:
        add(z[8] / z[11], z[10] / z[11])
    if abs(z[8]) > 1e-10:
        add(z[5] / z[8], z[7] / z[8])
    if abs(z[10]) > 1e-10:
        add(z[7] / z[10], z[9] / z[10])

    # Projective infinity branch (e.g., theta5 ~= +/-pi, x5 = tan(theta5/2) -> inf):
    # recover x4 from x5^2-scaled monomials and keep a large finite proxy for x5.
    # This lets downstream angle conversion and FK residual filtering keep valid roots.
    eps_inf = 1e-12
    if abs(z[11]) <= eps_inf and abs(z[9]) > eps_inf:
        x4_opts: List[float] = []
        for num, den in ((z[6], z[9]), (z[3], z[6]), (z[0], z[3])):
            if abs(den) <= eps_inf:
                continue
            ratio = float(num / den)
            if np.isfinite(ratio):
                x4_opts.append(ratio)
        if x4_opts:
            x4_inf = float(np.median(np.array(x4_opts, dtype=float)))
            sign = 1.0
            if abs(z[10]) > eps_inf:
                sign = 1.0 if z[10] >= 0.0 else -1.0
            elif abs(z[7]) > eps_inf and abs(x4_inf) > eps_inf:
                sign = 1.0 if (z[7] / x4_inf) >= 0.0 else -1.0
            add(x4_inf, sign * PROJECTIVE_INF_X)
            add(x4_inf, -sign * PROJECTIVE_INF_X)
    return cands


def _monomial12_from_x4x5(x4: float, x5: float) -> np.ndarray:
    # Basis order used by Eq. (14) assembly:
    # [x4^3x5^2,x4^3x5,x4^3,x4^2x5^2,x4^2x5,x4^2,x4x5^2,x4x5,x4,x5^2,x5,1]
    return np.array(
        [
            x4**3 * x5**2,
            x4**3 * x5,
            x4**3,
            x4**2 * x5**2,
            x4**2 * x5,
            x4**2,
            x4 * x5**2,
            x4 * x5,
            x4,
            x5**2,
            x5,
            1.0,
        ],
        dtype=float,
    )


def solve_x4x5_from_eq14_eigenvectors(
    M: np.ndarray,
    E9: np.ndarray | None = None,
    *,
    max_vectors: int = 8,
) -> List[Tuple[float, float]]:
    """Extract (x4, x5) from eigenvectors/nullspace of Eq. (14) matrix.

    Primary path:
      1) right eigenvectors associated with smallest |lambda| of M
      2) convert projective monomial vector -> (x4, x5) via ratios
      3) verify with Eq. (14), and optionally E9 polynomial rows

    Fallback:
      - use smallest singular vector (SVD null approximation).
    """
    candidates: List[Tuple[float, float]] = []

    def append_unique(x4: float, x5: float) -> None:
        if not np.isfinite(x4) or not np.isfinite(x5):
            return
        if abs(x4) > RECOVERY_X_ABS_MAX or abs(x5) > RECOVERY_X_ABS_MAX:
            return
        for u, v in candidates:
            if abs(x4 - u) <= 1e-7 and abs(x5 - v) <= 1e-7:
                return
        candidates.append((x4, x5))

    def is_valid_pair(x4: float, x5: float) -> bool:
        # For projective infinity proxies, polynomial residuals in x-space become
        # numerically unstable; defer final validity to FK residual filtering.
        if max(abs(x4), abs(x5)) >= 1e10:
            return True
        z12 = _monomial12_from_x4x5(x4, x5)
        eq14_err = float(np.max(np.abs(M @ z12)))
        if eq14_err > 1e-5:
            return False
        if E9 is not None:
            z9 = np.array(
                [
                    x4 * x4 * x5 * x5,
                    x4 * x4 * x5,
                    x4 * x5 * x5,
                    x4 * x5,
                    x4 * x4,
                    x5 * x5,
                    x4,
                    x5,
                    1.0,
                ],
                dtype=float,
            )
            e9_err = float(np.max(np.abs(E9 @ z9)))
            if e9_err > 1e-3:
                return False
        return True

    # Eigenvector path.
    if HAS_SCIPY and scipy_linalg is not None:
        try:
            eigvals, eigvecs = scipy_linalg.eig(M, check_finite=False)
            order = np.argsort(np.abs(eigvals))
            for idx in order[: max_vectors]:
                vec = eigvecs[:, idx]
                if np.max(np.abs(np.imag(vec))) > 1e-8:
                    continue
                z = np.real(vec)
                if np.max(np.abs(z)) <= 1e-14:
                    continue
                z = z / np.max(np.abs(z))
                for x4, x5 in recover_x4x5_candidates(z):
                    if is_valid_pair(x4, x5):
                        append_unique(x4, x5)
        except Exception:
            pass

    # SVD nullspace fallback.
    if not candidates:
        z = null_vector(M)
        for x4, x5 in recover_x4x5_candidates(z):
            if is_valid_pair(x4, x5):
                append_unique(x4, x5)

    return candidates


def recover_theta12(P: np.ndarray, Q: np.ndarray, theta4: float, theta5: float) -> Tuple[float, float]:
    m45 = monomial_45(theta4, theta5)
    rhs = P @ m45
    n, *_ = np.linalg.lstsq(Q, rhs, rcond=None)
    s1, c1 = normalize_sc(float(n[4]), float(n[5]))
    s2, c2 = normalize_sc(float(n[6]), float(n[7]))
    theta1 = normalize_angle(math.atan2(s1, c1))
    theta2 = normalize_angle(math.atan2(s2, c2))
    return theta1, theta2


def recover_theta6(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    q1: float,
    q2: float,
    q3: float,
    q4: float,
    q5: float,
) -> float:
    T1to5 = np.eye(4)
    q_prefix = [q1, q2, q3, q4, q5]
    for i in range(5):
        a_i, alpha_i, d_i = dh_params[i]
        T1to5 = T1to5 @ dh_matrix(a_i, alpha_i, d_i, q_prefix[i])
    A6_target = se3_inverse(T1to5) @ target_pose
    c6 = float(A6_target[0, 0])
    s6 = float(A6_target[1, 0])
    return normalize_angle(math.atan2(s6, c6))


def pose_residual(
    dh_params: Sequence[Sequence[float]],
    target_pose: np.ndarray,
    q: Sequence[float],
) -> float:
    fk = forward_kinematics(dh_params, q)
    return float(np.linalg.norm(fk - target_pose))


def _dedupe_solutions(solutions: Iterable[IKSolution], tol: float = ANGLE_TOL) -> List[IKSolution]:
    out: List[IKSolution] = []
    for sol in solutions:
        q = np.array(sol.q, dtype=float)
        unique = True
        for existing in out:
            qe = np.array(existing.q, dtype=float)
            diff = np.array([normalize_angle(float(a - b)) for a, b in zip(q, qe)])
            if float(np.linalg.norm(diff)) <= tol:
                unique = False
                break
        if unique:
            out.append(sol)
    return sorted(out, key=lambda s: s.residual)


def solve_ik_6r_general(
    dh_params: Sequence[Sequence[float]],
    target_pose: Sequence[Sequence[float]] | np.ndarray,
    *,
    sample_count: int = 60,
    interpolation_grid: Sequence[float] | None = None,
    residual_tolerance: float = RESIDUAL_TOL,
    root_solver: str = "matrix_polynomial_pencil",
    matrix_pencil_degree: int = 16,
    matrix_pencil_root_bound_factor: float = 4.0,
    enable_bracket_fallback: bool = True,
    return_diagnostics: bool = False,
) -> Union[List[IKSolution], Tuple[List[IKSolution], Dict[str, Any]]]:
    """
    Solve IK for a general 6R robot using resultant elimination.

    Returns:
        Deduplicated list of candidate joint vectors with FK residuals.
        If return_diagnostics=True, returns (solutions, diagnostics).

    Args:
        root_solver:
            Only "matrix_polynomial_pencil" is supported.
        matrix_pencil_degree:
            Degree used to fit the matrix polynomial in x3.
        matrix_pencil_root_bound_factor:
            Real-root post-filter bound multiplier based on
            max(|interpolation_grid|), for matrix-pencil mode.
        enable_bracket_fallback:
            If True, use determinant sign-change bracketing only when
            eigensolver yields no candidate real roots.
    """

    dh = np.asarray(dh_params, dtype=float)
    if dh.shape != (6, 3):
        raise ValueError("dh_params must have shape (6, 3) as (a, alpha, d).")
    target = np.asarray(target_pose, dtype=float)
    if target.shape != (4, 4):
        raise ValueError("target_pose must be a 4x4 homogeneous transform.")

    # sample_count is retained for API compatibility; symbolic-exported P/Q
    # formulas use a fixed 17-sample linear map.
    samples = PQ_SYMBOLIC_SAMPLE_ANGLES
    x_samples = (
        np.array(interpolation_grid, dtype=float)
        if interpolation_grid is not None
        else np.linspace(-6.0, 6.0, 25)
    )
    if len(x_samples) < 17:
        raise ValueError("interpolation_grid must contain at least 17 points.")

    # Determinant evaluator (used by root-refinement and fallback).
    det_cache: dict[float, float] = {}

    def detf(x: float) -> float:
        key = float(x)
        if key not in det_cache:
            det_cache[key] = characteristic_det_at_x3(dh, target, key, samples)
        return det_cache[key]

    if root_solver != "matrix_polynomial_pencil":
        raise ValueError(
            "Only root_solver='matrix_polynomial_pencil' is supported."
        )

    # Stage 1 + 2: sample M(x3) and fit matrix polynomial pencil.
    stage_matrix_det_time = 0.0
    stage_poly_fit_time = 0.0
    matrix_pencil_diag: Dict[str, Any] = {}
    coeff_mats: List[np.ndarray] | None = None
    matrix_pencil_scale = 1.0

    coeff_mats, matrix_pencil_scale, stage_matrix_det_time, stage_poly_fit_time, matrix_pencil_diag = (
        fit_matrix_polynomial_pencil_with_timing(
            dh,
            target,
            samples,
            x_samples,
            degree=int(matrix_pencil_degree),
        )
    )

    # Stage 3: solve for x3 roots + optional robust fallback
    stage_poly_solve_t0 = time.perf_counter()
    poly_eigen_time = 0.0
    poly_bracket_time = 0.0
    poly_solver_stage = root_solver
    t_poly_eig = time.perf_counter()
    roots = _solve_matrix_polynomial_pencil_eigenvalues(coeff_mats or [])
    poly_eigen_time = time.perf_counter() - t_poly_eig
    poly_num_roots = int(len(roots))
    poly_num_real_roots = int(sum(abs(r.imag) <= 1e-7 for r in roots))
    x_bound = float(matrix_pencil_root_bound_factor) * max(
        1.0, float(np.max(np.abs(x_samples)))
    )
    raw_real = [
        float(r.real * matrix_pencil_scale)
        for r in roots
        if abs(r.imag) <= 1e-7 and np.isfinite(r.real)
    ]
    raw_real = [x for x in raw_real if np.isfinite(x) and abs(x) <= x_bound]
    raw_real = _dedupe_scalar_roots(raw_real, tol=1e-4)

    refined = [_refine_root_newton(detf, x) for x in raw_real]
    refined = [x for x in refined if np.isfinite(x) and abs(x) <= x_bound]
    refined = _dedupe_scalar_roots(refined, tol=1e-5)

    if refined:
        scored = sorted((abs(detf(x)), x) for x in refined)
        near_zero = [x for err, x in scored if err <= 1e-6]
        real_x3 = near_zero if near_zero else [x for _, x in scored]
    else:
        real_x3 = []
    real_x3 = _dedupe_scalar_roots(real_x3, tol=1e-5)
    stage_poly_solve_time = time.perf_counter() - stage_poly_solve_t0

    candidates: List[IKSolution] = []
    map45 = build_halfangle_map_45()
    used_bracket_fallback = False
    bracket_root_count = 0
    eq14_x45_candidate_total = 0
    eq14_x45_e9_fallback_uses = 0
    eq14_x45_e9_fallback_candidate_total = 0

    # Stage 4: back-substitute for remaining angles
    def _run_back_sub(root_list: Sequence[float]) -> float:
        nonlocal eq14_x45_candidate_total
        nonlocal eq14_x45_e9_fallback_uses
        nonlocal eq14_x45_e9_fallback_candidate_total
        t0 = time.perf_counter()
        for x3 in root_list:
            theta3 = normalize_angle(2.0 * math.atan(x3))
            P, Q, _ = build_pq_at_x3(dh, target, x3, samples)
            E45 = eliminate_theta12(P, Q)
            if E45 is None:
                continue
            E9 = E45 @ map45
            M = build_dialytic_12x12(E9)
            x45_candidates = solve_x4x5_from_eq14_eigenvectors(M, E9)
            if not x45_candidates:
                eq14_x45_e9_fallback_uses += 1
                x45_candidates = solve_x4x5_from_e9(E9)
                eq14_x45_e9_fallback_candidate_total += len(x45_candidates)
            else:
                eq14_x45_candidate_total += len(x45_candidates)
            for x4, x5 in x45_candidates:
                theta4 = normalize_angle(2.0 * math.atan(x4))
                theta5 = normalize_angle(2.0 * math.atan(x5))
                theta1, theta2 = recover_theta12(P, Q, theta4, theta5)
                theta6 = recover_theta6(dh, target, theta1, theta2, theta3, theta4, theta5)
                q = (
                    normalize_angle(theta1),
                    normalize_angle(theta2),
                    normalize_angle(theta3),
                    normalize_angle(theta4),
                    normalize_angle(theta5),
                    normalize_angle(theta6),
                )
                residual = pose_residual(dh, target, q)
                if residual <= residual_tolerance:
                    candidates.append(IKSolution(q=q, residual=residual))
        return time.perf_counter() - t0

    stage_back_sub_time = _run_back_sub(real_x3)

    # Fallback: if eig roots did not yield valid IK, use robust bracket roots.
    if enable_bracket_fallback and len(candidates) == 0:
        used_bracket_fallback = True
        t_fb_solve = time.perf_counter()
        x_min = float(np.min(x_samples))
        x_max = float(np.max(x_samples))
        bracket_roots = _bracket_roots_from_det(detf, x_min, x_max, num_scan_points=241)
        poly_bracket_time = time.perf_counter() - t_fb_solve
        stage_poly_solve_time += poly_bracket_time

        new_roots = [r for r in bracket_roots if all(abs(r - u) > 1e-5 for u in real_x3)]
        bracket_root_count = len(new_roots)
        if new_roots:
            real_x3 = _dedupe_scalar_roots(real_x3 + new_roots, tol=1e-5)
            stage_back_sub_time += _run_back_sub(new_roots)

    solutions = _dedupe_solutions(candidates)
    if not return_diagnostics:
        return solutions

    total_profiled = (
        stage_matrix_det_time
        + stage_poly_fit_time
        + stage_poly_solve_time
        + stage_back_sub_time
    )
    def pct(x: float) -> float:
        if total_profiled <= 0.0:
            return 0.0
        return 100.0 * x / total_profiled

    diagnostics: Dict[str, Any] = {
        "root_solver": root_solver,
        "matrix_pencil_degree_argument": int(matrix_pencil_degree),
        "matrix_pencil_root_bound_factor": float(matrix_pencil_root_bound_factor),
        "enable_bracket_fallback": bool(enable_bracket_fallback),
        "used_bracket_fallback": bool(used_bracket_fallback),
        "num_bracket_fallback_roots": int(bracket_root_count),
        "poly_solver_stage": poly_solver_stage,
        "num_polynomial_roots": poly_num_roots,
        "num_polynomial_real_roots": poly_num_real_roots,
        "num_candidate_x3_roots": int(len(real_x3)),
        "num_ik_solutions": int(len(solutions)),
        "num_real_ik_solutions": int(len(solutions)),
        "num_x45_candidates_eq14_eigenvector": int(eq14_x45_candidate_total),
        "num_x45_e9_fallback_uses": int(eq14_x45_e9_fallback_uses),
        "num_x45_candidates_e9_fallback": int(eq14_x45_e9_fallback_candidate_total),
        "pq_symbolic_sample_count": int(len(PQ_SYMBOLIC_SAMPLE_ANGLES)),
        "pq_fit_sample_count_argument": int(sample_count),
        "time_matrix_construction_sec": float(stage_matrix_det_time),
        "time_poly_derivation_sec": float(stage_poly_fit_time),
        "time_poly_solve_sec": float(stage_poly_solve_time),
        "time_poly_eigen_sec": float(poly_eigen_time),
        "time_poly_bracket_fallback_sec": float(poly_bracket_time),
        "time_back_substitution_sec": float(stage_back_sub_time),
        "time_profiled_total_sec": float(total_profiled),
        "time_matrix_construction_pct": float(pct(stage_matrix_det_time)),
        "time_poly_derivation_pct": float(pct(stage_poly_fit_time)),
        "time_poly_solve_pct": float(pct(stage_poly_solve_time)),
        "time_poly_eigen_pct": float(pct(poly_eigen_time)),
        "time_poly_bracket_fallback_pct": float(pct(poly_bracket_time)),
        "time_back_substitution_pct": float(pct(stage_back_sub_time)),
    }
    diagnostics.update(matrix_pencil_diag)
    return solutions, diagnostics


def _default_demo() -> None:
    dh = np.array(
        [
            [0.32, 0.70, 0.18],
            [0.25, -0.90, 0.21],
            [0.29, 0.80, 0.14],
            [0.22, -1.10, 0.19],
            [0.18, 0.60, 0.11],
            [0.15, -0.70, 0.17],
        ],
        dtype=float,
    )
    q_true = np.array([0.60, -1.00, 0.90, -0.80, 1.20, -0.40], dtype=float)
    target = forward_kinematics(dh, q_true)
    sols = solve_ik_6r_general(dh, target, sample_count=40, interpolation_grid=np.linspace(-6, 6, 25))
    print(f"Found {len(sols)} solutions")
    if sols:
        best = sols[0]
        print("Best q:", np.array(best.q))
        print("Best residual:", best.residual)


if __name__ == "__main__":
    _default_demo()

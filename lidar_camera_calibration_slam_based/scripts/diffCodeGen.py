import sympy

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_matrix(ai, aj, ak, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        print("_AXES2TUPLE ERROR!!!")

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = sympy.sin(ai), sympy.sin(aj), sympy.sin(ak)
    ci, cj, ck = sympy.cos(ai), sympy.cos(aj), sympy.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = sympy.eye(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M

def compose_matrix(angles=None, translate=None):
    M = sympy.eye(4)
    if translate is not None:
        M[:3, 3] = sympy.Matrix(translate[0:3])
    if angles is not None:
        R = euler_matrix(angles[0], angles[1], angles[2])
        M[0:3, 0:3] = R[0:3, 0:3]
    return M

def getCameraModelDiff(K):
    T = sympy.MatrixSymbol('T', 6, 1)
    P_L = sympy.MatrixSymbol('P_L', 4, 1)
    K = sympy.MatrixSymbol('K', 3, 3)
    T_C_L = compose_matrix(angles = T[3:6], translate = T[0:3])[0:3, :]
    P_C = K * T_C_L * P_L

    Jacobian = sympy.Matrix(P_C).jacobian(T)
    return Jacobian

from sympy.utilities.codegen import *
[(c_name, c_code), (h_name, c_header)] = codegen([('Jacobian_P_T', J)], 'C')

f = open(c_name, 'w')
f.write(c_code)
f.close()
f = open(h_name, 'w')
f.write(c_header)
f.close()

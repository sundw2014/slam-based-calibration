import sympy
def euler_matrix(ai, aj, ak):
    si, sj, sk = sympy.sin(ai), sympy.sin(aj), sympy.sin(ak)
    ci, cj, ck = sympy.cos(ai), sympy.cos(aj), sympy.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = sympy.matrices.eye(4)
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
    M = sympy.matrices.eye(4)
    if translate is not None:
        M[:3, 3] = translate[:3]
    if angles is not None:
        R = euler_matrix(angles[0], angles[1], angles[2])
        M[0:3, 0:3] = R[0:3, 0:3]
    return M

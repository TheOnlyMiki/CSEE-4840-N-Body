# gen_input_s1e8m18.py
# Output 1: HEX file (S1E8M18)
# Output 2: DEC file (quantized decimal)

import numpy as np

# =========================
# User knobs
# =========================
N_PARTICLE    = 1024
POS_MIN       = -200.0
POS_MAX       =  200.0
VEL_INIT_ZERO = True
VEL_MIN       = -0.1
VEL_MAX       =  0.1
MASS_MIN      =  0.2
MASS_MAX      =  1.0
RANDOM_SEED   = 0

OUTPUT_HEX = "frame0_1024binit200_27bits.txt"

# =========================
# S1E8M18 helpers
# =========================
BIAS = 127
E_BITS = 8
M_BITS = 18
TOTAL_BITS = 27
E_MASK = (1 << E_BITS) - 1
M_MASK = (1 << M_BITS) - 1
SIGN_SHIFT = TOTAL_BITS - 1
EXP_SHIFT = M_BITS
U27_MASK = (1 << TOTAL_BITS) - 1

def f27_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M_MASK)

def f27_to_hex(u27: int) -> str:
    return f"{u27 & U27_MASK:07X}"

def f27_to_float(u27: int) -> float:
    if u27 == 0:
        return 0.0
    sign = (u27 >> SIGN_SHIFT) & 1
    exp  = (u27 >> EXP_SHIFT) & E_MASK
    mant = u27 & M_MASK
    if exp == 0:
        return 0.0
    val = (1.0 + mant / (1 << M_BITS)) * (2.0 ** (exp - BIAS))
    return -val if sign else val

def f27_from_float(x: float) -> int:
    if x == 0.0:
        return 0
    sign = 1 if x < 0 else 0
    ax = abs(x)

    m, e = np.frexp(ax)
    m2 = m * 2.0
    e2 = e - 1
    exp = e2 + BIAS

    if exp <= 0:
        return 0
    if exp >= 255:
        return f27_pack(sign, 254, M_MASK)

    frac = m2 - 1.0
    mant_f = frac * (1 << M_BITS)

    mant = int(np.floor(mant_f + 0.5))
    if mant == (1 << M_BITS):
        mant = 0
        exp += 1
        if exp >= 255:
            exp = 254
            mant = M_MASK

    return f27_pack(sign, exp, mant)

# =========================
# Main
# =========================
def main():
    np.random.seed(RANDOM_SEED)

    pos = np.random.uniform(POS_MIN, POS_MAX, size=(N_PARTICLE, 2))
    if VEL_INIT_ZERO:
        vel = np.zeros((N_PARTICLE, 2), dtype=np.float64)
    else:
        vel = np.random.uniform(VEL_MIN, VEL_MAX, size=(N_PARTICLE, 2))

    mass = np.random.uniform(MASS_MIN, MASS_MAX, size=(N_PARTICLE,))

    with open(OUTPUT_HEX, "w") as f_hex:

        f_hex.write("# i  px  py  vx  vy  m   (S1E8M18 hex)\n")

        for i in range(N_PARTICLE):

            # original float
            px_f = float(pos[i, 0])
            py_f = float(pos[i, 1])
            vx_f = float(vel[i, 0])
            vy_f = float(vel[i, 1])
            m_f  = float(mass[i])

            # quantize
            px_u = f27_from_float(px_f)
            py_u = f27_from_float(py_f)
            vx_u = f27_from_float(vx_f)
            vy_u = f27_from_float(vy_f)
            m_u  = f27_from_float(m_f)

            # write HEX
            f_hex.write(
                f"{i:4d}  "
                f"{f27_to_hex(px_u)}  {f27_to_hex(py_u)}  "
                f"{f27_to_hex(vx_u)}  {f27_to_hex(vy_u)}  "
                f"{f27_to_hex(m_u)}\n"
            )


    print(f"[OK] Generated:")
    print(f"     {OUTPUT_HEX}")
    print(f"     N={N_PARTICLE}")

if __name__ == "__main__":
    main()

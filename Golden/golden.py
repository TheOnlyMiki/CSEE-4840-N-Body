
# ## 3. Python Model for FP modules

# ### 3.1 FP27 Add
# fp27_add_model.py
# Standalone RTL-mimic model for FpAdd (S1E8M18 core, 27-bit)

import random

# ============================================================
# format constants
# ============================================================
U27_MASK = (1 << 27) - 1
SIGN_SHIFT = 26
EXP_SHIFT  = 18
E_MASK     = 0xFF
M18_MASK   = (1 << 18) - 1


def u27_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M18_MASK)


def u27_fields(u: int):
    u &= U27_MASK
    s = (u >> SIGN_SHIFT) & 1
    e = (u >> EXP_SHIFT) & E_MASK
    m = u & M18_MASK
    return s, e, m


def u27_hex(u: int) -> str:
    return f"{u & U27_MASK:07X}"


def rtl_frac18_from_u27(u: int) -> int:
    """
    Match RTL:
      assign A_f = {1'b1, iA[17:1]};
    => internal 18-bit mantissa used by RTL add
    """
    return (1 << 17) | ((u >> 1) & 0x1FFFF)


def lod37_shift_amt(x: int) -> int:
    """
    Match RTL shft_amt:
      pre_frac[36] ? 0 :
      pre_frac[35] ? 1 :
      ...
      pre_frac[0]  ? 36 :
                     37;
    """
    x &= (1 << 37) - 1
    if x == 0:
        return 37
    for bit in range(36, -1, -1):
        if (x >> bit) & 1:
            return 36 - bit
    return 37


def fp27_add_rtl(a: int, b: int, debug: bool = False):
    """
    Bit-exact mimic of RTL FpAdd.
    Functionally collapses 2 pipeline stages into one Python call.
    """
    a &= U27_MASK
    b &= U27_MASK

    sa, ea, _ = u27_fields(a)
    sb, eb, _ = u27_fields(b)

    A_f = rtl_frac18_from_u27(a)  # 18-bit
    B_f = rtl_frac18_from_u27(b)  # 18-bit

    # --------------------------------------------------------
    # stage 1 combinational
    # --------------------------------------------------------
    A_larger = 1 if ((ea > eb) or ((ea == eb) and (A_f > B_f))) else 0

    exp_diff_A = (int(eb) - int(ea)) & 0xFF   # 9-bit-ish spirit
    exp_diff_B = (int(ea) - int(eb)) & 0xFF
    larger_exp = eb if (eb > ea) else ea

    # RTL:
    # {1'b0, A_f, 18'b0}  -> 37 bits
    A_ext = (A_f << 18) & ((1 << 37) - 1)
    B_ext = (B_f << 18) & ((1 << 37) - 1)

    if A_larger:
        A_f_shifted = A_ext
    else:
        A_f_shifted = 0 if exp_diff_A > 35 else (A_ext >> exp_diff_A)

    if not A_larger:
        B_f_shifted = B_ext
    else:
        B_f_shifted = 0 if exp_diff_B > 35 else (B_ext >> exp_diff_B)

    if ((sa ^ sb) & A_larger):
        pre_sum = (A_f_shifted - B_f_shifted) & ((1 << 37) - 1)
    elif ((sa ^ sb) & (1 - A_larger)):
        pre_sum = (B_f_shifted - A_f_shifted) & ((1 << 37) - 1)
    else:
        pre_sum = (A_f_shifted + B_f_shifted) & ((1 << 37) - 1)

    # --------------------------------------------------------
    # stage 1 registers
    # --------------------------------------------------------
    buf_pre_sum    = pre_sum
    buf_larger_exp = larger_exp
    buf_A_e_zero   = (ea == 0)
    buf_B_e_zero   = (eb == 0)
    buf_A          = a
    buf_B          = b
    buf_oSum_s     = sa if A_larger else sb

    # --------------------------------------------------------
    # stage 2 combinational / output logic
    # --------------------------------------------------------
    pre_frac = buf_pre_sum
    shft_amt = lod37_shift_amt(pre_frac)

    # RTL:
    # pre_frac_shft = {pre_frac,17'b0} << (shft_amt+1)
    # uflow_shift   = {pre_frac,17'b0} << (shft_amt)
    pre_frac_54  = (pre_frac << 17) & ((1 << 54) - 1)
    pre_frac_shft = (pre_frac_54 << (shft_amt + 1)) & ((1 << 54) - 1)
    uflow_shift   = (pre_frac_54 << shft_amt) & ((1 << 54) - 1)

    oSum_f = (pre_frac_shft >> 36) & M18_MASK
    oSum_e = (int(buf_larger_exp) - int(shft_amt) + 1) & 0xFF

    # RTL:
    # assign underflow = ~uflow_shift[53];
    underflow = 1 if (((uflow_shift >> 53) & 1) == 0) else 0

    if buf_A_e_zero and buf_B_e_zero:
        out = 0
    elif buf_A_e_zero:
        out = buf_B
    elif buf_B_e_zero:
        out = buf_A
    elif underflow:
        out = 0
    elif pre_frac == 0:
        out = 0
    else:
        out = u27_pack(buf_oSum_s, oSum_e, oSum_f)

    if debug:
        return {
            "a": a,
            "b": b,
            "sa": sa,
            "sb": sb,
            "ea": ea,
            "eb": eb,
            "A_f": A_f,
            "B_f": B_f,
            "A_larger": A_larger,
            "exp_diff_A": exp_diff_A,
            "exp_diff_B": exp_diff_B,
            "larger_exp": larger_exp,
            "A_f_shifted": A_f_shifted,
            "B_f_shifted": B_f_shifted,
            "pre_sum": pre_sum,
            "buf_pre_sum": buf_pre_sum,
            "pre_frac": pre_frac,
            "shft_amt": shft_amt,
            "pre_frac_54": pre_frac_54,
            "pre_frac_shft": pre_frac_shft,
            "uflow_shift": uflow_shift,
            "oSum_f": oSum_f,
            "oSum_e": oSum_e,
            "underflow": underflow,
            "buf_oSum_s": buf_oSum_s,
            "out": out,
            "out_hex": u27_hex(out),
        }
    return out


def make_case_file(path: str, n_random: int = 500):
    cases = []

    # --------------------------------------------------------
    # directed cases
    # --------------------------------------------------------
    directed = [
        # zeros
        (0x0000000, 0x0000000),
        (0x0000000, 0x3FC0000),
        (0x3FC0000, 0x0000000),

        # same-sign additions
        (0x3FC0000, 0x3FC0000),
        (0x4040000, 0x3FC0000),
        (0x4040000, 0x4040000),

        # sign-opposite / cancellation-ish
        (0x4440000, 0xC040000),
        (0xC040000, 0x4040000),
        (0x4040000, 0xC040000),
        (0x4040000, 0xC040001),  # test bit0-ish weirdness
        (0x3FC0000, 0xBFC0000),  # exact cancel
        (0x3FC0002, 0xBFC0000),  # near cancel

        # same exponent / different frac
        (0x3FC0001, 0x3FC0000),
        (0x3FC0002, 0x3FC0004),
        (0x7E7FFFF, 0x3FC0000),

        # large exponent diff
        (0x7C40000, 0x0440000),
        (0x0440000, 0x7C40000),

        # random looking hand-picked
        (0x11B31C6, 0x6AB377F),
        (0x390A3CF, 0x0F3EFAC),
        (0x4F2FC12, 0x0899422),
        (0x2C8993B, 0x4E9E669),
    ]
    cases.extend(directed)

    # --------------------------------------------------------
    # random normals / zeros
    # --------------------------------------------------------
    def rand_u27():
        s = random.getrandbits(1)
        e = 0 if random.random() < 0.08 else random.randint(1, 254)
        m = random.getrandbits(18)
        return u27_pack(s, e, m)

    for _ in range(n_random):
        a = rand_u27()
        b = rand_u27()
        cases.append((a, b))

    with open(path, "w") as f:
        for a, b in cases:
            y = fp27_add_rtl(a, b)
            f.write(f"{u27_hex(a)} {u27_hex(b)} {u27_hex(y)}\n")

    print(f"Wrote {len(cases)} cases to {path}")


# if __name__ == "__main__":
#     make_case_file("fpadd_cases.txt", n_random=500)


# ### 3.2 FP27 Mul
# fp27_mul_model.py
# Standalone RTL-mimic model for FpMul (S1E8M18 core, 27-bit)

import random

# ============================================================
# format constants
# ============================================================
U27_MASK = (1 << 27) - 1
SIGN_SHIFT = 26
EXP_SHIFT  = 18
E_MASK     = 0xFF
M18_MASK   = (1 << 18) - 1

# NOTE:
# RTL uses:
#   A_f = {1'b1, iA[17:1]}
# so bit0 is ignored by the multiplier datapath.


def u27_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M18_MASK)


def u27_fields(u: int):
    u &= U27_MASK
    s = (u >> SIGN_SHIFT) & 1
    e = (u >> EXP_SHIFT) & E_MASK
    m = u & M18_MASK
    return s, e, m


def u27_hex(u: int) -> str:
    return f"{u & U27_MASK:07X}"


def rtl_frac18_from_u27(u: int) -> int:
    """
    Match RTL:
      assign A_f = {1'b1, iA[17:1]};
    Effective 18-bit mantissa used internally:
      bit17 = hidden 1
      bit16:0 = stored bits [17:1]
    """
    return (1 << 17) | ((u >> 1) & 0x1FFFF)


def fp27_mul_rtl(a: int, b: int, debug: bool = False):
    """
    Bit-exact mimic of RTL FpMul:
      oProd_s = A_s ^ B_s
      pre_prod_frac = A_f * B_f
      pre_prod_exp  = A_e + B_e
      if pre_prod_frac[35]:
          oProd_e = pre_prod_exp - 126
          oProd_f = pre_prod_frac[34:17]
      else:
          oProd_e = pre_prod_exp - 127
          oProd_f = pre_prod_frac[33:16]
      underflow = pre_prod_exp < 128
      if underflow or A_e==0 or B_e==0: out=0
    """
    a &= U27_MASK
    b &= U27_MASK

    sa, ea, _ = u27_fields(a)
    sb, eb, _ = u27_fields(b)

    A_f = rtl_frac18_from_u27(a)
    B_f = rtl_frac18_from_u27(b)

    oProd_s = sa ^ sb
    pre_prod_frac = A_f * B_f   # 36-bit
    pre_prod_exp  = ea + eb     # 9-bit in RTL spirit

    if ((pre_prod_frac >> 35) & 1) == 1:
        oProd_e = (pre_prod_exp - 126) & 0xFF
        oProd_f = (pre_prod_frac >> 17) & M18_MASK   # [34:17]
        path = "hi"
    else:
        oProd_e = (pre_prod_exp - 127) & 0xFF
        oProd_f = (pre_prod_frac >> 16) & M18_MASK   # [33:16]
        path = "lo"

    underflow = pre_prod_exp < 0x80

    if underflow or (ea == 0) or (eb == 0):
        out = 0
    else:
        out = u27_pack(oProd_s, oProd_e, oProd_f)

    if debug:
        return {
            "a": a,
            "b": b,
            "sa": sa,
            "sb": sb,
            "ea": ea,
            "eb": eb,
            "A_f": A_f,
            "B_f": B_f,
            "pre_prod_frac": pre_prod_frac,
            "pre_prod_exp": pre_prod_exp,
            "oProd_s": oProd_s,
            "oProd_e": oProd_e,
            "oProd_f": oProd_f,
            "underflow": underflow,
            "path": path,
            "out": out,
            "out_hex": u27_hex(out),
        }
    return out


def make_case_file(path: str, n_random: int = 200):
    """
    Write:
      a_hex b_hex expected_hex
    """
    cases = []

    # -------- directed cases --------
    directed = [
        (0x0000000, 0x0000000),
        (0x0000000, 0x3FC0000),
        (0x3FC0000, 0x0000000),
        (0x3FC0000, 0x3FC0000),
        (0x4040000, 0x3FC0000),
        (0x4040000, 0x4040000),
        (0x43C0000, 0x3FC0000),
        (0x7E7FFFF, 0x3FC0000),
        (0x3FC0001, 0x3FC0001),  # test bit0 ignored or not
        (0x3FC0002, 0x3FC0002),
        (0x7C40000, 0x0440000),  # possible underflow-ish
        (0x4440000, 0xC040000),  # sign mix
        (0xC040000, 0xC040000),
    ]
    cases.extend(directed)

    # -------- random normals / zeros --------
    for _ in range(n_random):
        def rand_u27():
            s = random.getrandbits(1)
            # mostly normal exp, sometimes zero
            e = 0 if random.random() < 0.08 else random.randint(1, 254)
            m = random.getrandbits(18)
            return u27_pack(s, e, m)

        a = rand_u27()
        b = rand_u27()
        cases.append((a, b))

    with open(path, "w") as f:
        f.write("# a_hex b_hex expected_hex\n")
        for a, b in cases:
            y = fp27_mul_rtl(a, b)
            f.write(f"{u27_hex(a)} {u27_hex(b)} {u27_hex(y)}\n")

    print(f"Wrote {len(cases)} cases to {path}")


# if __name__ == "__main__":
#     make_case_file("fpmul_cases.txt", n_random=500)


# ### 3.3 FP27 InvSqrt
# fp27_invsqrt_model.py
import random

# ------------------------------------------------------------
# import your already-validated standalone models
# ------------------------------------------------------------
# from fp27_mul_model import fp27_mul_rtl
# from fp27_add_model import fp27_add_rtl

# ------------------------------------------------------------
# format constants
# ------------------------------------------------------------
U27_MASK    = (1 << 27) - 1
SIGN_SHIFT  = 26
EXP_SHIFT   = 18
E_MASK      = 0xFF
M18_MASK    = (1 << 18) - 1

# from your current model
MAGIC_C = 49920718   # 27'd49920718
ADD_K   = 33423360   # 27'd33423360


def u27_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M18_MASK)


def u27_fields(u: int):
    u &= U27_MASK
    s = (u >> SIGN_SHIFT) & 1
    e = (u >> EXP_SHIFT) & E_MASK
    m = u & M18_MASK
    return s, e, m


def u27_hex(u: int) -> str:
    return f"{u & U27_MASK:07X}"


def u27_neg(u: int) -> int:
    """Match FpNegate behavior for normal finite-style custom float."""
    u &= U27_MASK
    #if u == 0:
    #   return 0
    return u ^ (1 << SIGN_SHIFT)


def fast_inv_sqrt_u27_rtl(i_x: int, debug: bool = False) -> int:
    i_x &= U27_MASK
    w_sign, w_exp, w_man = u27_fields(i_x)

    # Stage 1
    w_s12_c = (MAGIC_C - (i_x >> 1)) & U27_MASK

    # IMPORTANT:
    # RTL does NOT flush exp==0 here.
    # It literally does {A_s, A_e-8'd1, A_f}, so exponent wraps.
    w_s12_b = u27_pack(w_sign, (w_exp - 1) & E_MASK, w_man)

    # Stage 1 mul
    w_s12_a = fp27_mul_rtl(w_s12_c, w_s12_c)

    # Stage 2 mul
    w_s23_a = fp27_mul_rtl(w_s12_b, w_s12_a)
    w_s23_b = w_s12_c

    # Stage 3 add: iA({~add_3_in[26], add_3_in[25:0]}), iB(ADD_K)
    w_s45_a = fp27_add_rtl(u27_neg(w_s23_a), ADD_K)
    w_s45_b = w_s23_b

    # Final mul
    out = fp27_mul_rtl(w_s45_b, w_s45_a)

    if debug:
        return {
            "i_x": i_x,
            "w_sign": w_sign,
            "w_exp": w_exp,
            "w_man": w_man,
            "w_s12_c": w_s12_c,
            "w_s12_b": w_s12_b,
            "w_s12_a": w_s12_a,
            "w_s23_a": w_s23_a,
            "w_s23_b": w_s23_b,
            "w_s45_a": w_s45_a,
            "w_s45_b": w_s45_b,
            "out": out,
            "out_hex": f"{out:07X}",
        }
    return out


def make_case_file(path: str, n_random: int = 500):
    cases = []

    # --------------------------------------------------------
    # directed cases
    # --------------------------------------------------------
    directed = [
        0x0000000,  # zero
        0x3FC0000,  # 1.0-ish
        0x4040000,  # 2.0-ish
        0x3C40000,  # 0.5-ish
        0x43C0000,  # 8.0-ish
        0x44C0000,
        0x7E7FFFF,  # large normal-ish
        0x0440000,  # tiny normal-ish
        0x3FC0001,  # bit0 perturb
        0x3FC0002,
        0x4040001,
        0x7C40000,
        0x0100000,
        0x1000000,
        0x2000000,
    ]
    for x in directed:
        cases.append(x)

    # --------------------------------------------------------
    # random normals / zeros
    # --------------------------------------------------------
    def rand_u27():
        s = 0  # invsqrt input should usually be non-negative r2/r2e
        e = 0 if random.random() < 0.08 else random.randint(1, 254)
        m = random.getrandbits(18)
        return u27_pack(s, e, m)

    for _ in range(n_random):
        cases.append(rand_u27())

    with open(path, "w") as f:
        for x in cases:
            y = fast_inv_sqrt_u27_rtl(x)
            f.write(f"{u27_hex(x)} {u27_hex(y)}\n")

    print(f"Wrote {len(cases)} cases to {path}")


# if __name__ == "__main__":
#     make_case_file("fpinvsqrt_cases.txt", n_random=500)



# ## 4. Full Hardware Model

# v4real_chiplike_f187_output.py
# IO:   27-bit custom float S1E8M18 (sign 1, exp 8, mant 18)
# CORE: 27-bit custom float S1E8M18 (sign 1, exp 8, mant 18)
#
# Outputs (N_FRAMES steps, frames numbered 0,1,2,...):
#   OUT_ROOT/
#     frame_chip_input/    : chip input frames (HEX S1E8M18). frame0.txt copied from INPUT_FILE
#     accel_chip_output/   : chip output accelerations (HEX S1E8M18). accel0.txt, accel1.txt, ...
#     accel_core27_output/ : internal accumulated accel (HEX S1E8M18). accel27_0.txt, accel27_1.txt, ...
#     frame_host_output/   : host frames (DECIMAL). frame0.txt, frame1.txt, ...
#
# Physics convention:
# - Chip accel is "chiplike" and MUST NOT include G0.
# - Host update uses:
#     K_HOST = G0 * DT   (only used in v-update)
#     v += a_chip * K_HOST
#     p += v * DT        (DT remains here; do NOT multiply G0 into this)
#
# IMPORTANT (BIT-EXACT CORE):
# - Core ops (add/mul/neg/fast_inv_sqrt) NEVER use float.
# - Core arithmetic is imported from standalone RTL-mimic models.
# - Host boundary quantization still uses TRUNC; denorm flush to 0.
# - Overflow clamps to max finite (exp=254, mant=all-1).

import os
import shutil
import numpy as np

# ============================================================
# IMPORT BIT-EXACT CORE (match standalone RTL-mimic modules)
# ============================================================
# The functions fp27_add_rtl, fp27_mul_rtl, and fast_inv_sqrt_u27_rtl
# are defined in earlier cells. We will assign them directly to aliases.
fp27_add = fp27_add_rtl
fp27_mul = fp27_mul_rtl
fast_inv_sqrt_u27 = fast_inv_sqrt_u27_rtl

# ============================================================
# Top-level knobs
# ============================================================
INPUT_FILE = "../Hardware/tb/frame_input/frame0_256binit200_27bits.txt"
OUT_ROOT = "../Hardware/tb/output/eps025_v4real_chiplike_256binit200"
# INPUT_FILE = "frame0_8binit10.txt"
# OUT_ROOT = "eps025_v4real_chiplike_8binit10"

DIR_FRAME_HOST   = os.path.join(OUT_ROOT, "frame_host_output")     # decimal
DIR_FRAME_CHIP   = os.path.join(OUT_ROOT, "frame_chip_input")      # hex (S1E8M18)
DIR_ACCEL_CHIP   = os.path.join(OUT_ROOT, "accel_chip_output")     # hex (S1E8M18)
DIR_ACCEL_CORE27 = os.path.join(OUT_ROOT, "accel_core27_output")   # hex (S1E8M18)

N_FRAMES = 4

# Host integrator settings
DT = 0.1
G0 = 10.0
K_HOST = G0 * DT

HOST_DTYPE = np.float32

# EPS^2 bit-exact constant in u27.
# Matches RTL: localparam logic [26:0] EPSILON_SQUARE = {1'b0, 8'd125, 18'd0};
# This encodes 2^(125 - 127) = 0.25, so epsilon = 0.5.
EPS_SQUARE_U27 = (0 << 26) | (125 << 18) | 0

# ============================================================
# Shared float layout params
# ============================================================
BIAS = 127
E_BITS = 8
E_MASK = (1 << E_BITS) - 1

# ============================================================
# CORE format: S1E8M18 (27-bit)
# ============================================================
M18_BITS = 18
U27_MASK = (1 << 27) - 1
SIGN27_SHIFT = 26
EXP27_SHIFT = M18_BITS
M18_MASK = (1 << M18_BITS) - 1

def u27_fields(u27: int):
    u27 &= U27_MASK
    s = (u27 >> SIGN27_SHIFT) & 1
    e = (u27 >> EXP27_SHIFT) & E_MASK
    m = u27 & M18_MASK
    return s, e, m

def u27_pack(s: int, e: int, m: int) -> int:
    return ((s & 1) << SIGN27_SHIFT) | ((e & E_MASK) << EXP27_SHIFT) | (m & M18_MASK)

def u27_neg(u27: int) -> int:
    # FIX: match RTL FpNegate exactly: {~A_s, A_e, A_f}
    # RTL unconditionally flips bit26, no zero guard.
    return (u27 ^ (1 << SIGN27_SHIFT)) & U27_MASK

def u27_to_hex(u27: int) -> str:
    return f"{u27 & U27_MASK:07X}"

def u27_from_hex(h: str) -> int:
    return int(h, 16) & U27_MASK

def u27_to_float(u27: int) -> float:
    """For host-side view only."""
    u27 &= U27_MASK
    if u27 == 0:
        return 0.0
    s, e, m = u27_fields(u27)
    if e == 0:
        return -0.0 if s else 0.0
    return (-1.0 if s else 1.0) * (2.0 ** (int(e) - BIAS)) * (1.0 + (m / (1 << M18_BITS)))

def u27_from_float_trunc(x: float) -> int:
    """TRUNC into S1E8M18. Denorm flush to 0. Overflow clamp."""
    if x == 0.0 or not np.isfinite(x):
        if x == 0.0:
            return 0
        sign = 1 if np.signbit(x) else 0
        return u27_pack(sign, 254, M18_MASK)

    sign = 1 if x < 0 else 0
    ax = abs(x)
    if ax == 0.0:
        return 0

    m, e = np.frexp(ax)
    m2 = m * 2.0
    e2 = e - 1
    exp = int(e2 + BIAS)

    if exp <= 0:
        return 0
    if exp >= 255:
        return u27_pack(sign, 254, M18_MASK)

    mant = int(np.floor((m2 - 1.0) * (1 << M18_BITS)))
    if mant < 0:
        mant = 0
    if mant > M18_MASK:
        mant = M18_MASK

    return u27_pack(sign, exp, mant)

# ============================================================
# File IO helpers
# ============================================================
def load_frame0_u27(path: str):
    idx, px, py, vx, vy, mm = [], [], [], [], [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if (not line) or line.startswith("#"):
                continue
            parts = line.split()
            idx.append(int(parts[0]))
            px.append(u27_from_hex(parts[1]))
            py.append(u27_from_hex(parts[2]))
            vx.append(u27_from_hex(parts[3]))
            vy.append(u27_from_hex(parts[4]))
            mm.append(u27_from_hex(parts[5]))
    return (
        np.array(idx, dtype=np.int32),
        np.array(px, dtype=np.uint32),
        np.array(py, dtype=np.uint32),
        np.array(vx, dtype=np.uint32),
        np.array(vy, dtype=np.uint32),
        np.array(mm, dtype=np.uint32),
    )

def write_frame_chip_hex(path: str, idx, px_u27, py_u27, vx_u27, vy_u27, m_u27):
    with open(path, "w") as f:
        f.write("# i  px  py  vx  vy  m   (S1E8M18 hex)\n")
        for i in range(len(idx)):
            f.write(
                f"{int(idx[i])}  {u27_to_hex(int(px_u27[i]))}  {u27_to_hex(int(py_u27[i]))}  "
                f"{u27_to_hex(int(vx_u27[i]))}  {u27_to_hex(int(vy_u27[i]))}  {u27_to_hex(int(m_u27[i]))}\n"
            )

def write_accel_core27_hex(path: str, ax_u27, ay_u27):
    with open(path, "w") as f:
        f.write("# i  ax27  ay27   (S1E8M18 hex)\n")
        for i in range(len(ax_u27)):
            f.write(f"{i}  {u27_to_hex(int(ax_u27[i]))}  {u27_to_hex(int(ay_u27[i]))}\n")

def write_frame_host_decimal(path: str, idx, px_f, py_f, vx_f, vy_f, m_f, g0: float, dt: float, k_host: float):
    with open(path, "w") as f:
        f.write(f"# G0 = {g0:.12e}\n")
        f.write(f"# DT = {dt:.12e}\n")
        f.write(f"# K_HOST = G0*DT = {k_host:.12e}\n")
        f.write("# i  px  py  vx  vy  m   (decimal float)\n")
        for i in range(len(idx)):
            f.write(
                f"{int(idx[i])}  {float(px_f[i]): .12e}  {float(py_f[i]): .12e}  "
                f"{float(vx_f[i]): .12e}  {float(vy_f[i]): .12e}  {float(m_f[i]): .12e}\n"
            )

# ============================================================
# Core compute: chip hex -> internal u27 -> accel
# ============================================================
def compute_acc_once_u27io_accum27(px_u27, py_u27, m_u27, eps2_u27: int):
    """
    - input is already 27-bit S1E8M18
    - all ops in 27-bit bit-exact core
    - sum_ax/sum_ay accumulated in 27-bit
    """
    N = len(px_u27)
    ax_u27 = np.zeros(N, dtype=np.uint32)
    ay_u27 = np.zeros(N, dtype=np.uint32)

    px = [int(v) & U27_MASK for v in px_u27]
    py = [int(v) & U27_MASK for v in py_u27]
    mm = [int(v) & U27_MASK for v in m_u27]

    for i in range(N):
        sum_ax = 0
        sum_ay = 0

        for j in range(N):
            if j == i:
                continue

            dx = fp27_add(px[j], u27_neg(px[i]))
            dy = fp27_add(py[j], u27_neg(py[i]))

            dx2 = fp27_mul(dx, dx)
            dy2 = fp27_mul(dy, dy)
            r2  = fp27_add(dx2, dy2)
            r2e = fp27_add(r2, eps2_u27)

            # FIX: match RTL two_body_core.v pipeline mul order exactly:
            #   c12: s2 = s*s  AND  t = m2*s  (parallel)
            #   c13: k  = t*s2   →  (m2*s) * (s*s)
            # NOT the same as m2 * (s*s*s) due to FpMul bit0 truncation
            s  = fast_inv_sqrt_u27(r2e)
            s2 = fp27_mul(s, s)       # s²
            t  = fp27_mul(mm[j], s)   # m₂·s  (parallel with s2 in RTL)
            k  = fp27_mul(t, s2)      # (m₂·s)·s²

            term_x = fp27_mul(k, dx)
            term_y = fp27_mul(k, dy)

            sum_ax = fp27_add(sum_ax, term_x)
            sum_ay = fp27_add(sum_ay, term_y)

        ax_u27[i] = sum_ax & U27_MASK
        ay_u27[i] = sum_ay & U27_MASK

    return ax_u27, ay_u27

# ============================================================
# Main loop
# ============================================================
def main():
    os.makedirs(DIR_FRAME_HOST, exist_ok=True)
    os.makedirs(DIR_FRAME_CHIP, exist_ok=True)
    os.makedirs(DIR_ACCEL_CHIP, exist_ok=True)
    os.makedirs(DIR_ACCEL_CORE27, exist_ok=True)

    print("Imported core modules:")
    print("  fp27_add         ->", fp27_add.__module__)
    print("  fp27_mul         ->", fp27_mul.__module__)
    print("  fast_inv_sqrt_u27->", fast_inv_sqrt_u27.__module__)

    eps2_u27 = EPS_SQUARE_U27

    # Load frame0 (27-bit hex)
    idx, px0_u27, py0_u27, vx0_u27, vy0_u27, m0_u27 = load_frame0_u27(INPUT_FILE)

    # Copy frame0 into frame_chip_input/frame0.txt
    frame0_chip_path = os.path.join(DIR_FRAME_CHIP, "frame0.txt")
    try:
        shutil.copyfile(INPUT_FILE, frame0_chip_path)
    except Exception:
        write_frame_chip_hex(frame0_chip_path, idx, px0_u27, py0_u27, vx0_u27, vy0_u27, m0_u27)

    # Host state (float) initialized from frame0 hex --- FP32
    px_f = np.array([u27_to_float(int(v)) for v in px0_u27], dtype=HOST_DTYPE)
    py_f = np.array([u27_to_float(int(v)) for v in py0_u27], dtype=HOST_DTYPE)
    vx_f = np.array([u27_to_float(int(v)) for v in vx0_u27], dtype=HOST_DTYPE)
    vy_f = np.array([u27_to_float(int(v)) for v in vy0_u27], dtype=HOST_DTYPE)
    m_f  = np.array([u27_to_float(int(v)) for v in m0_u27],  dtype=HOST_DTYPE)

    # Chip state (hex) initialized from frame0
    px_u27 = px0_u27.copy()
    py_u27 = py0_u27.copy()
    vx_u27 = vx0_u27.copy()
    vy_u27 = vy0_u27.copy()
    m_u27  = m0_u27.copy()

    # Dump host decimal frame0
    host0_path = os.path.join(DIR_FRAME_HOST, "frame0.txt")
    write_frame_host_decimal(host0_path, idx, px_f, py_f, vx_f, vy_f, m_f, G0, DT, K_HOST)

    # Produce frames 0..N_FRAMES, and accels 0..N_FRAMES-1
    for k in range(N_FRAMES):
        # 1) Chip computes accel for current frame k (NO G0 inside)
        ax_u27, ay_u27 = compute_acc_once_u27io_accum27(
            px_u27, py_u27, m_u27, eps2_u27
        )

        accel_path = os.path.join(DIR_ACCEL_CHIP, f"accel{k}.txt")
        write_accel_core27_hex(accel_path, ax_u27, ay_u27)

        accel27_path = os.path.join(DIR_ACCEL_CORE27, f"accel27_{k}.txt")
        write_accel_core27_hex(accel27_path, ax_u27, ay_u27)

        # 2) Host update (FP32) using accel from chip (apply K_HOST only here)
        ax_f = np.array([u27_to_float(int(v)) for v in ax_u27], dtype=HOST_DTYPE)
        ay_f = np.array([u27_to_float(int(v)) for v in ay_u27], dtype=HOST_DTYPE)

        k_host_f = HOST_DTYPE(K_HOST)
        dt_f     = HOST_DTYPE(DT)

        vx_f = (vx_f + ax_f * k_host_f).astype(HOST_DTYPE, copy=False)
        vy_f = (vy_f + ay_f * k_host_f).astype(HOST_DTYPE, copy=False)
        px_f = (px_f + vx_f * dt_f).astype(HOST_DTYPE, copy=False)
        py_f = (py_f + vy_f * dt_f).astype(HOST_DTYPE, copy=False)

        # 3) Quantize updated host frame back to 27-bit chip hex for next frame using TRUNC
        px_u27 = np.array([u27_from_float_trunc(float(x)) for x in px_f], dtype=np.uint32)
        py_u27 = np.array([u27_from_float_trunc(float(x)) for x in py_f], dtype=np.uint32)
        vx_u27 = np.array([u27_from_float_trunc(float(x)) for x in vx_f], dtype=np.uint32)
        vy_u27 = np.array([u27_from_float_trunc(float(x)) for x in vy_f], dtype=np.uint32)

        # mass stays constant
        m_u27 = m_u27

        # 4) Write next chip input frame and host decimal frame
        chip_path = os.path.join(DIR_FRAME_CHIP, f"frame{k+1}.txt")
        write_frame_chip_hex(chip_path, idx, px_u27, py_u27, vx_u27, vy_u27, m_u27)

        host_path = os.path.join(DIR_FRAME_HOST, f"frame{k+1}.txt")
        write_frame_host_decimal(host_path, idx, px_f, py_f, vx_f, vy_f, m_f, G0, DT, K_HOST)

    print(f"[OK] Done. OUT_ROOT={OUT_ROOT}")
    print(f"     EPS_SQUARE_U27=0x{eps2_u27:07X}")
    print(f"     G0={G0}  DT={DT}  K_HOST={K_HOST}")
    print(f"     HOST_DTYPE={HOST_DTYPE}")
    print(f"     Frames written: frame0..frame{N_FRAMES}")
    print(f"     Accels written: accel0..accel{N_FRAMES-1}")
    print(f"     Core27 written: accel27_0..accel27_{N_FRAMES-1}")

# if __name__ == "__main__":
#     main()


# ============================================================
# TB-compatible one-shot golden output
# Match tb_core_accel input/output behavior
# ============================================================

TB_INPUT_FILE = "../Hardware/tb/frame_input/frame0_256binit200_27bits.txt"
TB_GOLDEN_OUT = "../Hardware/tb/output/golden_accel_256binit200_27bits.txt"

def run_tb_compatible_golden():
    idx, px_u27, py_u27, vx_u27, vy_u27, m_u27 = load_frame0_u27(TB_INPUT_FILE)

    ax_u27, ay_u27 = compute_acc_once_u27io_accum27(
        px_u27,
        py_u27,
        m_u27,
        EPS_SQUARE_U27
    )

    with open(TB_GOLDEN_OUT, "w") as f:
        f.write("# i ax ay (S1E8M18 hex)\n")
        for i in range(len(idx)):
            f.write(f"{int(idx[i]):4d}  {u27_to_hex(int(ax_u27[i]))}  {u27_to_hex(int(ay_u27[i]))}\n")

    print(f"[OK] Wrote TB-compatible golden output: {TB_GOLDEN_OUT}")

if __name__ == "__main__":
    run_tb_compatible_golden()


# ## 5. Compare
import os

# ============================================================
# USER CONFIG
# ============================================================
RTL_FILE = "../Hardware/tb/output/eps025_accel_256binit200_27bits.txt"
PY_FILE  = "../Hardware/tb/output/golden_accel_256binit200_27bits.txt"
# RTL_FILE = "eps025_temp_27bits_8binit10.txt"
# PY_FILE  = "eps025_v4real_chiplike_8binit10/accel_core27_output/accel27_0.txt"

# if you want, save mismatch report
REPORT_FILE = "compare_27bit_report.txt"

# ============================================================
# HELPERS
# ============================================================
U27_MASK = (1 << 27) - 1

def popcount(x: int) -> int:
    return bin(x).count("1")

def load_27bit_txt(path: str):
    """
    Expected format:
      # i  ax27  ay27 ...
      0  3ABCDE1  0123456
      1  1234567  7654321
    Returns:
      dict[idx] = (ax27_int, ay27_int)
    """
    data = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            idx = int(parts[0])
            ax = int(parts[1], 16) & U27_MASK
            ay = int(parts[2], 16) & U27_MASK
            data[idx] = (ax, ay)
    return data

# ============================================================
# MAIN
# ============================================================
def main():
    rtl = load_27bit_txt(RTL_FILE)
    py  = load_27bit_txt(PY_FILE)

    rtl_keys = set(rtl.keys())
    py_keys  = set(py.keys())

    only_in_rtl = sorted(rtl_keys - py_keys)
    only_in_py  = sorted(py_keys - rtl_keys)
    common      = sorted(rtl_keys & py_keys)

    lines = []
    lines.append(f"RTL_FILE = {RTL_FILE}")
    lines.append(f"PY_FILE  = {PY_FILE}")
    lines.append("")

    if only_in_rtl:
        lines.append(f"indices only in RTL: {only_in_rtl}")
    if only_in_py:
        lines.append(f"indices only in PY : {only_in_py}")
    if only_in_rtl or only_in_py:
        lines.append("")

    mismatch_count = 0
    max_ax_bits = -1
    max_ay_bits = -1
    max_ax_idx = None
    max_ay_idx = None

    total_ax_bits = 0
    total_ay_bits = 0

    for i in common:
        rtl_ax, rtl_ay = rtl[i]
        py_ax,  py_ay  = py[i]

        diff_ax = rtl_ax ^ py_ax
        diff_ay = rtl_ay ^ py_ay

        bits_ax = popcount(diff_ax)
        bits_ay = popcount(diff_ay)

        total_ax_bits += bits_ax
        total_ay_bits += bits_ay

        if bits_ax > max_ax_bits:
            max_ax_bits = bits_ax
            max_ax_idx = i

        if bits_ay > max_ay_bits:
            max_ay_bits = bits_ay
            max_ay_idx = i

        if diff_ax != 0 or diff_ay != 0:
            mismatch_count += 1
            lines.append(
                f"{i:4d} | "
                f"{rtl_ax:07X},{rtl_ay:07X} -> {py_ax:07X},{py_ay:07X} | "
                f"{bits_ax},{bits_ay}"
            )

    lines.append("")
    lines.append("Summary")
    lines.append("-------")
    lines.append(f"common indices      : {len(common)}")
    lines.append(f"mismatch rows       : {mismatch_count}")
    lines.append(f"total ax bit diffs  : {total_ax_bits}")
    lines.append(f"total ay bit diffs  : {total_ay_bits}")
    lines.append(f"max ax bit diff     : {max_ax_bits}  (index {max_ax_idx})")
    lines.append(f"max ay bit diff     : {max_ay_bits}  (index {max_ay_idx})")

    report = "\n".join(lines)
    print(report)

    with open(REPORT_FILE, "w") as f:
        f.write(report)

    print(f"\nWrote report to: {REPORT_FILE}")

# if __name__ == "__main__":
#     main()

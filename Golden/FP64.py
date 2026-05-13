# ## 2. FP64 Golden Reference


# fp64_reference_loop.py
# Pure float64 N-body reference with "chip-accel convention":
#   - accel_chip_output: DOES NOT include G0
#   - host update uses:  K_HOST = G0 * DT   (v += a_chip * K_HOST)
#
# Output structure same as chiplike version (3 folders)

import os
import shutil
import numpy as np

# ============================================================
# User knobs
# ============================================================
INPUT_FILE = "../Hardware/tb/frame_input/frame0_256binit200.txt"  # f187 hex input

OUT_ROOT = "./Hardware/tb/output/tempfp64_256binit200_outputs"
DIR_FRAME_HOST = os.path.join(OUT_ROOT, "frame_host_output")
DIR_FRAME_CHIP = os.path.join(OUT_ROOT, "frame_chip_input")
DIR_ACCEL_CHIP = os.path.join(OUT_ROOT, "accel_chip_output")

N_FRAMES = 10
DT = 0.1
G0 = 10.0

# IMPORTANT: this is epsilon^2 added to r^2, use very small value for closer-to-realistic accel (but not too small to cause underflow issues in fp64)
EPS_SQ = 2.0 ** (-27)

# Host update gain (this is where G0 goes)
K_HOST = G0 * DT

# ============================================================
# S1E8M7 helpers (same as before)
# ============================================================
BIAS = 127
E_BITS = 8
M_BITS = 7
TOTAL_BITS = 16
E_MASK = (1 << E_BITS) - 1
M_MASK = (1 << M_BITS) - 1
SIGN_SHIFT = TOTAL_BITS - 1
EXP_SHIFT = M_BITS
U16_MASK = (1 << TOTAL_BITS) - 1

def f16_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M_MASK)

def f16_to_hex(u16: int) -> str:
    return f"{u16 & U16_MASK:04X}"

def f16_from_hex(h: str) -> int:
    return int(h, 16) & U16_MASK

def f16_to_float(u16: int) -> float:
    if u16 == 0:
        return 0.0
    sign = (u16 >> SIGN_SHIFT) & 1
    exp  = (u16 >> EXP_SHIFT) & E_MASK
    mant = u16 & M_MASK
    if exp == 0:
        return 0.0
    val = (1.0 + mant / (1 << M_BITS)) * (2.0 ** (exp - BIAS))
    return -val if sign else val

def f16_from_float(x: float) -> int:
    if x == 0.0:
        return 0
    sign = 1 if x < 0 else 0
    ax = abs(x)

    m, e = np.frexp(ax)  # ax = m * 2^e, m in [0.5, 1)
    m2 = m * 2.0
    e2 = e - 1
    exp = e2 + BIAS

    if exp <= 0:
        return 0
    if exp >= 255:
        return f16_pack(sign, 254, M_MASK)

    frac = m2 - 1.0
    mant_f = frac * (1 << M_BITS)
    mant = int(np.floor(mant_f + 0.5))  # round-to-nearest

    if mant == (1 << M_BITS):
        mant = 0
        exp += 1
        if exp >= 255:
            exp = 254
            mant = M_MASK

    return f16_pack(sign, exp, mant)

# ============================================================
# IO helpers
# ============================================================
def load_frame0_hex(path: str):
    idx, px, py, vx, vy, mm = [], [], [], [], [], []
    with open(path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            idx.append(int(parts[0]))
            px.append(f16_to_float(f16_from_hex(parts[1])))
            py.append(f16_to_float(f16_from_hex(parts[2])))
            vx.append(f16_to_float(f16_from_hex(parts[3])))
            vy.append(f16_to_float(f16_from_hex(parts[4])))
            mm.append(f16_to_float(f16_from_hex(parts[5])))
    return (np.array(idx, dtype=np.int32),
            np.array(px, dtype=np.float64),
            np.array(py, dtype=np.float64),
            np.array(vx, dtype=np.float64),
            np.array(vy, dtype=np.float64),
            np.array(mm, dtype=np.float64))

def write_frame_host(path, idx, px, py, vx, vy, m):
    with open(path, "w") as f:
        f.write("# i px py vx vy m (float64)\n")
        for i in range(len(idx)):
            f.write(f"{idx[i]} {px[i]: .12e} {py[i]: .12e} "
                    f"{vx[i]: .12e} {vy[i]: .12e} {m[i]: .12e}\n")

def write_frame_chip(path, idx, px, py, vx, vy, m):
    with open(path, "w") as f:
        f.write("# i px py vx vy m (S1E8M7 hex)\n")
        for i in range(len(idx)):
            f.write(f"{idx[i]} "
                    f"{f16_to_hex(f16_from_float(px[i]))} "
                    f"{f16_to_hex(f16_from_float(py[i]))} "
                    f"{f16_to_hex(f16_from_float(vx[i]))} "
                    f"{f16_to_hex(f16_from_float(vy[i]))} "
                    f"{f16_to_hex(f16_from_float(m[i]))}\n")

def write_accel_chip(path, idx, ax, ay):
    # accel is "chip convention" (NO G0), but written as S1E8M7 hex for chip-output compare
    with open(path, "w") as f:
        f.write("# i ax ay (S1E8M7 hex)   [chip accel: NO G0]\n")
        for i in range(len(idx)):
            f.write(f"{idx[i]} "
                    f"{f16_to_hex(f16_from_float(ax[i]))} "
                    f"{f16_to_hex(f16_from_float(ay[i]))}\n")

# ============================================================
# FP64 N-body compute (chip accel: NO G0)
# ============================================================
def compute_acc_chip(px, py, m):
    """
    Compute chip-style acceleration (NO G0):
      a_chip = sum_j m[j] * d / (r^2 + eps^2)^(3/2)
    """
    N = len(px)
    ax = np.zeros(N, dtype=np.float64)
    ay = np.zeros(N, dtype=np.float64)

    for i in range(N):
        axi = 0.0
        ayi = 0.0
        pxi = px[i]
        pyi = py[i]
        for j in range(N):
            if i == j:
                continue
            dx = px[j] - pxi
            dy = py[j] - pyi
            r2 = dx*dx + dy*dy + EPS_SQ
            inv3 = 1.0 / (r2 ** 1.5)
            factor = m[j] * inv3  # <-- NO G0 here
            axi += factor * dx
            ayi += factor * dy
        ax[i] = axi
        ay[i] = ayi
    return ax, ay

# ============================================================
# Main
# ============================================================
def main():
    os.makedirs(DIR_FRAME_HOST, exist_ok=True)
    os.makedirs(DIR_FRAME_CHIP, exist_ok=True)
    os.makedirs(DIR_ACCEL_CHIP, exist_ok=True)

    idx, px, py, vx, vy, m = load_frame0_hex(INPUT_FILE)

    # frame0 outputs
    write_frame_host(os.path.join(DIR_FRAME_HOST, "frame0.txt"),
                     idx, px, py, vx, vy, m)
    shutil.copyfile(INPUT_FILE,
                    os.path.join(DIR_FRAME_CHIP, "frame0.txt"))

    for k in range(N_FRAMES):
        # 1) chip accel (no G0)
        ax_chip, ay_chip = compute_acc_chip(px, py, m)

        # 2) write chip accel as hex (for compare with RTL chip output)
        write_accel_chip(os.path.join(DIR_ACCEL_CHIP, f"accel{k}.txt"),
                         idx, ax_chip, ay_chip)

        # 3) host update uses K_HOST = G0 * DT
        vx = vx + ax_chip * K_HOST
        vy = vy + ay_chip * K_HOST
        px = px + vx * DT
        py = py + vy * DT

        # 4) write next frame (float64 host) + chip-input hex view
        write_frame_host(os.path.join(DIR_FRAME_HOST, f"frame{k+1}.txt"),
                         idx, px, py, vx, vy, m)

        write_frame_chip(os.path.join(DIR_FRAME_CHIP, f"frame{k+1}.txt"),
                         idx, px, py, vx, vy, m)

    print("[OK] FP64 reference generated (chip-accel convention, K_HOST used).")
    print(f"     EPS_SQ={EPS_SQ}  DT={DT}  G0={G0}  K_HOST={K_HOST}")

if __name__ == "__main__":
    main()

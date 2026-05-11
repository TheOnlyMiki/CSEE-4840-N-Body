#!/usr/bin/env python3
"""
Golden model for nbody_control + four_core_wrapper + nbody_integrator.

Default output matches Hardware/tb/tb_nbody_control.sv:
    # i x y vx vy ax ay (S1E8M18 hex)
       0  0123456  0ABCDEF  0000000  0000000  0123000  0ABC000

The model reuses the bit-exact FP/core helpers from golden.py, then applies the
same control-level ordering: 16-body tiles, four 4-lane groups per tile, and
ascending lane writeback. It also updates the body memory with the RTL
integrator semantics so --gap and --first-step match nbody_control.
"""

import argparse
import os
import sys
from typing import Iterable, List, Optional, Tuple


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import golden as core  # noqa: E402


U27_MASK = (1 << 27) - 1
SIGN_SHIFT = 26
EXP_SHIFT = 18
E_MASK = 0xFF
M18_MASK = (1 << 18) - 1
TILE_STRIDE = 16
GROUP_SIZE = 4
EPS_SQUARE_U27 = (0 << 26) | (125 << 18) | 0

DEFAULT_INPUT = os.path.join(
    REPO_ROOT, "Hardware", "tb", "frame_input", "frame0_1024binit200_27bits.txt"
)
DEFAULT_STATE_OUT = os.path.join(
    REPO_ROOT, "Hardware", "tb", "output", "golden_control_state_1024binit200_27bits.txt"
)


class BodyState:
    def __init__(
        self,
        x: List[int],
        y: List[int],
        vx: List[int],
        vy: List[int],
        m: List[int],
        ax: List[int],
        ay: List[int],
    ) -> None:
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.m = m
        self.ax = ax
        self.ay = ay


def u27_hex(value: int) -> str:
    return f"{value & U27_MASK:07X}"


def u27_from_hex(text: str) -> int:
    return int(text, 16) & U27_MASK


def u27_pack(sign: int, exp: int, mant: int) -> int:
    return ((sign & 1) << SIGN_SHIFT) | ((exp & E_MASK) << EXP_SHIFT) | (mant & M18_MASK)


def u27_fields(value: int) -> Tuple[int, int, int]:
    value &= U27_MASK
    return (value >> SIGN_SHIFT) & 1, (value >> EXP_SHIFT) & E_MASK, value & M18_MASK


def half_step_accel(value: int) -> int:
    """Match nbody_control's exponent-only half-step acceleration adjustment."""
    sign, exp, mant = u27_fields(value)
    half_exp = exp - 1 if exp > 1 else 0
    return u27_pack(sign, half_exp, mant)


def integrator_step(
    x: int,
    y: int,
    vx: int,
    vy: int,
    ax: int,
    ay: int,
    half_step: bool = False,
) -> Tuple[int, int, int, int]:
    """Match nbody_integrator.sv: vx'=vx+ax, x'=x+vx'."""
    if half_step:
        ax = half_step_accel(ax)
        ay = half_step_accel(ay)

    vx_new = core.fp27_add_rtl(vx, ax)
    vy_new = core.fp27_add_rtl(vy, ay)
    x_new = core.fp27_add_rtl(x, vx_new)
    y_new = core.fp27_add_rtl(y, vy_new)
    return x_new, y_new, vx_new, vy_new


def load_frame(path: str, max_bodies: int) -> Tuple[BodyState, int]:
    x = [0] * max_bodies
    y = [0] * max_bodies
    vx = [0] * max_bodies
    vy = [0] * max_bodies
    m = [0] * max_bodies
    highest_idx = -1

    with open(path, "r") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 6:
                raise ValueError(f"{path}:{line_no}: expected 'idx x y vx vy m'")

            idx = int(parts[0])
            if idx < 0 or idx >= max_bodies:
                continue

            x[idx] = u27_from_hex(parts[1])
            y[idx] = u27_from_hex(parts[2])
            vx[idx] = u27_from_hex(parts[3])
            vy[idx] = u27_from_hex(parts[4])
            m[idx] = u27_from_hex(parts[5])
            highest_idx = max(highest_idx, idx)

    return BodyState(x=x, y=y, vx=vx, vy=vy, m=m, ax=[0] * max_bodies, ay=[0] * max_bodies), highest_idx + 1


def compute_one_body_accel(state: BodyState, i: int, active_count: int) -> Tuple[int, int]:
    """Bit-exact model of one accumulated four_core_wrapper output lane."""
    sum_ax = 0
    sum_ay = 0

    for j in range(active_count):
        if j == i:
            continue

        dx = core.fp27_add_rtl(state.x[j], core.u27_neg(state.x[i]))
        dy = core.fp27_add_rtl(state.y[j], core.u27_neg(state.y[i]))

        dx2 = core.fp27_mul_rtl(dx, dx)
        dy2 = core.fp27_mul_rtl(dy, dy)
        r2 = core.fp27_add_rtl(dx2, dy2)
        r2e = core.fp27_add_rtl(r2, EPS_SQUARE_U27)

        s = core.fast_inv_sqrt_u27_rtl(r2e)
        s2 = core.fp27_mul_rtl(s, s)
        t = core.fp27_mul_rtl(state.m[j], s)
        k = core.fp27_mul_rtl(t, s2)

        term_x = core.fp27_mul_rtl(k, dx)
        term_y = core.fp27_mul_rtl(k, dy)
        sum_ax = core.fp27_add_rtl(sum_ax, term_x)
        sum_ay = core.fp27_add_rtl(sum_ay, term_y)

    return sum_ax & U27_MASK, sum_ay & U27_MASK


def control_write_order(active_count: int) -> Iterable[int]:
    """Yield body indices in the same tile/group/lane order as nbody_control."""
    for tile_base in range(0, active_count, TILE_STRIDE):
        for group in range(TILE_STRIDE // GROUP_SIZE):
            group_base = tile_base + group * GROUP_SIZE
            for lane in range(GROUP_SIZE):
                idx = group_base + lane
                if idx < active_count:
                    yield idx


def compute_accel_pass(state: BodyState, active_count: int) -> List[Tuple[int, int, int]]:
    writes: List[Tuple[int, int, int]] = []

    for i in control_write_order(active_count):
        ax, ay = compute_one_body_accel(state, i, active_count)
        state.ax[i] = ax
        state.ay[i] = ay
        writes.append((i, ax, ay))

    return writes


def integrate_pass(state: BodyState, active_count: int, half_step: bool) -> None:
    for i in range(active_count):
        x, y, vx, vy = integrator_step(
            state.x[i],
            state.y[i],
            state.vx[i],
            state.vy[i],
            state.ax[i],
            state.ay[i],
            half_step=half_step,
        )
        state.x[i] = x
        state.y[i] = y
        state.vx[i] = vx
        state.vy[i] = vy


def run_control_model(
    state: BodyState,
    active_count: int,
    gap: int,
    first_step: bool,
) -> Tuple[List[Tuple[int, int, int]], BodyState]:
    """
    Run the control-level model.

    Returns the first timestep's accel writes because tb_nbody_control.sv uses
    gap=1 and writes one flat accel output file. For gap>1, later timesteps are
    still integrated into state, but not appended to that flat file.
    """
    if gap <= 0 or active_count <= 0:
        return [], state

    first_writes: Optional[List[Tuple[int, int, int]]] = None
    for timestep in range(gap):
        writes = compute_accel_pass(state, active_count)
        if first_writes is None:
            first_writes = writes
        integrate_pass(state, active_count, half_step=first_step and timestep == 0)

    return first_writes or [], state


def write_accel_file(path: str, writes: Iterable[Tuple[int, int, int]]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("# i ax ay (S1E8M18 hex)\n")
        for idx, ax, ay in writes:
            f.write(f"{idx:4d}  {u27_hex(ax)}  {u27_hex(ay)}\n")


def write_frame_file(path: str, state: BodyState, active_count: int) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("# i  px  py  vx  vy  m   (S1E8M18 hex)\n")
        for i in range(active_count):
            f.write(
                f"{i:4d}  {u27_hex(state.x[i])}  {u27_hex(state.y[i])}  "
                f"{u27_hex(state.vx[i])}  {u27_hex(state.vy[i])}  {u27_hex(state.m[i])}\n"
            )


def write_state_file(path: str, state: BodyState, active_count: int) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("# i x y vx vy ax ay (S1E8M18 hex)\n")
        for i in range(active_count):
            f.write(
                f"{i:4d}  {u27_hex(state.x[i])}  {u27_hex(state.y[i])}  "
                f"{u27_hex(state.vx[i])}  {u27_hex(state.vy[i])}  "
                f"{u27_hex(state.ax[i])}  {u27_hex(state.ay[i])}\n"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a golden output for nbody_control.sv."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Input frame file.")
    parser.add_argument(
        "--output",
        default=DEFAULT_STATE_OUT,
        help="Output final x/y/vx/vy/ax/ay file matching tb_nbody_control.sv.",
    )
    parser.add_argument("--frame-output", default=None, help="Optional final updated frame output.")
    parser.add_argument(
        "--accel-output",
        default=None,
        help="Optional acceleration-only write stream file.",
    )
    parser.add_argument("--max-bodies", type=int, default=1024, help="Memory capacity.")
    parser.add_argument("--n-bodies", type=int, default=None, help="Active body count.")
    parser.add_argument("--gap", type=int, default=1, help="Number of timesteps to run.")
    parser.add_argument(
        "--first-step",
        action="store_true",
        help="Apply the initial leapfrog half-step kick on timestep 0.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    state, loaded_count = load_frame(args.input, args.max_bodies)
    active_count = args.n_bodies if args.n_bodies is not None else loaded_count
    active_count = max(0, min(active_count, args.max_bodies))

    writes, final_state = run_control_model(
        state=state,
        active_count=active_count,
        gap=args.gap,
        first_step=args.first_step,
    )
    write_state_file(args.output, final_state, active_count)

    if args.frame_output:
        write_frame_file(args.frame_output, final_state, active_count)

    if args.accel_output:
        write_accel_file(args.accel_output, writes)

    print(f"[OK] input        : {args.input}")
    print(f"[OK] n_bodies     : {active_count}")
    print(f"[OK] gap          : {args.gap}")
    print(f"[OK] first_step   : {int(args.first_step)}")
    print(f"[OK] state output : {args.output}")
    if args.frame_output:
        print(f"[OK] frame output : {args.frame_output}")
    if args.accel_output:
        print(f"[OK] accel output : {args.accel_output}")


if __name__ == "__main__":
    main()

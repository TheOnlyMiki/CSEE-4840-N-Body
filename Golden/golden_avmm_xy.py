#!/usr/bin/env python3
"""
Golden X/Y stream for the current nbody_accel_avmm hardware path.

This script takes the same six-column S1E8M18 frame input file that software
loads into the accelerator, runs the bit-level Python model of the current RTL,
and writes the X/Y positions that software reads from OUT_X/OUT_Y after DONE.

The default first-step behavior matches nbody_accel_avmm.sv: loading bodies via
VY_IN marks the next GO as the initial leapfrog half-step run.
"""

import argparse
import os
import sys
from typing import Optional


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import golden_control as hw  # noqa: E402


DEFAULT_INPUT = os.path.join(
    REPO_ROOT, "Hardware", "tb", "frame_input", "frame0_1024binit200_27bits.txt"
)
DEFAULT_OUTPUT = os.path.join(
    REPO_ROOT, "Hardware", "tb", "output", "golden_xy_1024binit200_27bits.txt"
)


def checked_active_count(loaded_count: int, requested_count: Optional[int], max_bodies: int) -> int:
    if requested_count is None:
        active_count = loaded_count
    else:
        active_count = requested_count

    if active_count <= 0:
        raise ValueError(f"n_bodies must be positive, got {active_count}")
    if active_count > max_bodies:
        raise ValueError(f"n_bodies={active_count} exceeds max_bodies={max_bodies}")
    if active_count > loaded_count:
        raise ValueError(
            f"n_bodies={active_count} but input only contains rows through index {loaded_count - 1}"
        )

    return active_count


def write_xy_file(path: str, input_path: str, active_count: int, gap: int, first_step: bool,
                  state: hw.BodyState) -> None:
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(path, "w") as f:
        f.write(f"# source {input_path}\n")
        f.write(f"# n_bodies {active_count}\n")
        f.write(f"# gap {gap}\n")
        f.write(f"# first_step {int(first_step)}\n")
        f.write("# i x y (S1E8M18 hex)\n")
        for i in range(active_count):
            f.write(f"{i:4d}  {hw.u27_hex(state.x[i])}  {hw.u27_hex(state.y[i])}\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate the golden OUT_X/OUT_Y stream returned by current "
            "nbody_accel_avmm hardware after one GO/DONE run."
        )
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Input frame txt file.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output X/Y txt file.")
    parser.add_argument("--max-bodies", type=int, default=1024, help="Hardware memory capacity.")
    parser.add_argument("--n-bodies", type=int, default=None, help="Active body count.")
    parser.add_argument("--gap", type=int, default=1, help="Hardware GAP register value.")
    parser.add_argument(
        "--no-first-step",
        action="store_true",
        help="Model a later GO without reloading bodies. Default models GO after body load.",
    )
    parser.add_argument(
        "--frame-output",
        default=None,
        help="Optional full final frame output with x/y/vx/vy/m.",
    )
    parser.add_argument(
        "--accel-output",
        default=None,
        help="Optional acceleration write stream output, matching tb_nbody_control.sv.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.max_bodies <= 0:
        raise ValueError(f"max_bodies must be positive, got {args.max_bodies}")
    if args.gap <= 0:
        raise ValueError(f"gap must be positive, got {args.gap}")

    state, loaded_count = hw.load_frame(args.input, args.max_bodies)
    active_count = checked_active_count(loaded_count, args.n_bodies, args.max_bodies)
    first_step = not args.no_first_step

    accel_writes, final_state = hw.run_control_model(
        state=state,
        active_count=active_count,
        gap=args.gap,
        first_step=first_step,
    )

    write_xy_file(
        path=args.output,
        input_path=args.input,
        active_count=active_count,
        gap=args.gap,
        first_step=first_step,
        state=final_state,
    )

    if args.frame_output:
        hw.write_frame_file(args.frame_output, final_state, active_count)

    if args.accel_output:
        hw.write_accel_file(args.accel_output, accel_writes)

    print(f"[OK] input        : {args.input}")
    print(f"[OK] n_bodies     : {active_count}")
    print(f"[OK] gap          : {args.gap}")
    print(f"[OK] first_step   : {int(first_step)}")
    print(f"[OK] xy output    : {args.output}")
    if args.frame_output:
        print(f"[OK] frame output : {args.frame_output}")
    if args.accel_output:
        print(f"[OK] accel output : {args.accel_output}")


if __name__ == "__main__":
    main()

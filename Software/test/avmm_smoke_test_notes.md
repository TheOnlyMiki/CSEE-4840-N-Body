# Minimal software smoke test for `vga_bitmap_avmm` and `nbody_accel_avmm`

## What this test checks

This first test is only meant to confirm that Linux software can reach the correct Avalon-MM address windows through the HPS lightweight bridge.

It checks:

1. The VGA bitmap peripheral can be written without crashing the HPS bus.
2. The VGA framebuffer address packing matches the hardware comment: `word_index = y * 20 + x / 32`, `bit_index = x % 32`.
3. The n-body accelerator control path responds to `N_BODIES`, `GAP`, `GO`, `DONE`, and `READ`.
4. A tiny one-body n-body run can be loaded through the input registers and read through `OUT_X` / `OUT_Y`.

This does **not** fully verify accelerator math correctness yet.

## Important limitation in the current RTL

`vga_bitmap_avmm.sv` always returns `readdata = 32'd0`, so software cannot read back framebuffer contents. The VGA test must be checked visually on the monitor.

`nbody_accel_avmm.sv` only exposes these readable registers:

| Word address | Register | Meaning |
|---:|---|---|
| `0x08` | `DONE` | bit 0 is high when the run has completed |
| `0x0A` | `OUT_X` | current output body's x position, low 27 bits valid |
| `0x0B` | `OUT_Y` | current output body's y position, low 27 bits valid; reading this increments the output pointer |

The input registers are write-only from the software point of view, so this smoke test cannot prove input write correctness by direct readback.

## Build

On the DE1-SoC Linux system:

```bash
gcc -O2 -Wall -Wextra -o avmm_smoke_test avmm_smoke_test.c
```

## Run

Use the Platform Designer address map to find each component's offset relative to the HPS lightweight bridge.

```bash
sudo ./avmm_smoke_test <nbody_offset_hex> <vga_offset_hex>
```

Example only:

```bash
sudo ./avmm_smoke_test 0x00000 0x01000
```

The program assumes the standard lightweight bridge base:

```c
#define LW_BRIDGE_BASE 0xFF200000u
#define LW_BRIDGE_SPAN 0x00200000u
```

## Expected result

For VGA, the monitor should show a black screen with a white border and a diagonal line.

For n-body, the zero-body test should show:

```text
[pass] DONE went high.
[pass] DONE cleared after READ=0.
```

For the one-body test, the program loads:

```text
x  = 1.0  = 0x01FC0000 in 27-bit S1E8M18
 y = 2.0  = 0x02000000 in 27-bit S1E8M18
m  = 1.0
vx = 0
vy = 0
```

Since there is only one body, self-interaction should be masked, so the output position is expected to stay approximately/exactly the same for this smoke case. If it does not match exactly, do not immediately assume the Avalon bus is broken; the next step is to check the accelerator/integrator behavior in simulation with the same input.

## Recommended next RTL improvement for easier software debug

Add one or two simple readable scratch/debug registers to each AVMM module. For example:

- `SCRATCH`: read/write, returns the last written value.
- `ID`: read-only fixed constant, such as `0x4E424F44` for `NBOD` and `0x56474142` for `VGAB`.

That would let software prove address decoding and read/write correctness before depending on the accelerator datapath or the VGA monitor.

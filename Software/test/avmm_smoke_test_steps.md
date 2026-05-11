# Running avmm_smoke_test on DE1-SoC

## Files needed on the board

Copy these files into one directory on the DE1-SoC, for example `/root/avmm_test`:

- `avmm_smoke_test.c`
- `avmm_frame_xy_dump.c`
- `Makefile`

## Build

```sh
cd /root/avmm_test
make
```

This produces:

```sh
./avmm_smoke_test
./avmm_frame_xy_dump
```

## Find your Avalon offsets

The smoke test assumes the HPS lightweight bridge base is:

```text
0xFF200000
```

You must pass each peripheral offset relative to that base.

For example, if Platform Designer says:

```text
nbody_accel_avmm_0 base = 0x00000000
vga_bitmap_avmm_0  base = 0x00010000
```

then run:

```sh
make run NBODY_OFFSET=0x00000000 VGA_OFFSET=0x00010000
```

You can find these offsets from one of these places:

1. Platform Designer Address Map tab.
2. Generated `soc_system.dts`.
3. `/proc/device-tree/...` after boot.
4. `/proc/iomem`, if a kernel driver reserved the region.

## Run

Example only:

```sh
make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
```

Expected behavior:

- Terminal prints the lightweight bridge base and the two offsets.
- VGA readback prints zero values because the current `vga_bitmap_avmm` RTL does not implement readback.
- VGA screen should show a border and diagonal pattern.
- N-body `DONE` should go high for the simple zero-body/one-body smoke tests.
- One-body output may warn if the integrator changes the position, but the test still confirms the basic read path.

## Dump one frame's X/Y output

After the smoke test passes, use `avmm_frame_xy_dump` to load one of the same
six-column frame txt files used by the SystemVerilog testbench, run the
accelerator once, and write X/Y positions for golden comparison:

```sh
make run-frame \
  NBODY_OFFSET=0x00000 \
  FRAME_INPUT=../../Hardware/tb/frame_input/frame0_1024binit200_27bits.txt \
  FRAME_OUTPUT=hw_xy_1024.txt \
  FRAME_N=1024 \
  FRAME_GAP=1
```

The output file format is:

```text
# i x y (S1E8M18 hex)
   0  20CE1A0  2156136
```

`FRAME_N` may be smaller than the frame file length if you want a faster
bring-up run using the first N bodies. `FRAME_GAP` is the accelerator's internal
timestep count between `DONE` pulses.

## If `/dev/mem` permission fails

Run as root. On the course image you are usually already root. Otherwise use:

```sh
sudo make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
```

## Important

Do not load a kernel driver that claims the same memory region while using this direct `/dev/mem` smoke test. For this first smoke test, we are bypassing the Lab 3 ioctl driver path.

# CSEE-4840 N-Body Accelerator

FPGA-accelerated N-body simulation for the DE1-SoC platform. The project combines a custom SystemVerilog N-body accelerator, an Avalon-MM VGA bitmap peripheral, Linux kernel drivers, userspace control/display code, direct hardware smoke tests, and Python golden models for RTL validation.

The accelerator stores up to 1024 bodies and computes simulation steps in hardware using a custom 27-bit floating-point format (`S1E8M18`). Software loads body state through an Avalon-MM register interface, starts the accelerator, reads updated positions, and renders the simulation to a packed 640x480 1-bpp VGA framebuffer.

## Repository Layout

```text
.
|-- Golden/                    Python golden/reference models
|-- Hardware/
|   |-- code/                  SystemVerilog accelerator and VGA peripherals
|   |-- ip/                    Platform Designer custom IP wrappers
|   |-- tb/                    SystemVerilog testbenches and test vectors
|   |-- Makefile               Quartus/Qsys/device-tree/preloader build flow
|   |-- soc_system.qsys        Platform Designer system
|   |-- soc_system.tcl         Quartus project generation script
|   `-- soc_system_top.sv      FPGA top level
|-- Software/
|   |-- *.c, *.h               Linux app, drivers, display, keyboard control
|   |-- test/                  Direct /dev/mem Avalon-MM smoke tests
|   `-- Makefile               Userspace app and kernel-module build
`-- CSEE4840_project_design_document.pdf
```

## Hardware Overview

The main RTL lives in `Hardware/code/`.

- `nbody_accel_avmm.sv` is the Avalon-MM top-level shell for the accelerator.
- `nbody_control.sv` sequences tiled N-body computation and integration.
- `four_core_wrapper.sv` evaluates four body lanes in parallel over 16-body tiles.
- `fourcore_bcj_datapath.sv`, `FpAdd.sv`, `FpMul.sv`, `FpInvSqrt.sv`, and `FpNegate.sv` implement the custom floating-point datapath.
- `nbody_mem.sv` stores positions, velocities, masses, and accelerations.
- `nbody_integrator.sv` updates position and velocity.
- `vga_bitmap_avmm.sv` exposes a packed 640x480 bitmap framebuffer over Avalon-MM.

The accelerator register map is documented at the top of `Hardware/code/nbody_accel_avmm.sv`.

## Prerequisites

Typical development uses:

- Intel Quartus / Platform Designer command-line tools
- Intel/Altera embedded command shell tools for `sopc2dts`, `dtc`, BSP, and ARM cross-compilation
- Synopsys VCS or another SystemVerilog simulator for RTL testbenches
- Python 3 for golden model generation
- GCC, Linux kernel headers, and `libusb-1.0` for the software app and drivers

On the DE1-SoC, log in as `root` before running commands that insert drivers or access FPGA peripherals through `/dev/mem` or device nodes.

## Hardware Build

Run hardware build commands from `Hardware/`.

```sh
cd Hardware
make qsys       # Generate Platform Designer output from soc_system.qsys
make project    # Generate Quartus project files and HPS SDRAM pin constraints
make quartus    # Compile the FPGA design and produce output_files/soc_system.sof
make rbf        # Convert the .sof into output_files/soc_system.rbf
make dtb        # Generate soc_system.dtb from soc_system.sopcinfo
```
Generated Quartus, Qsys, VCS, and local editor files are intentionally ignored by Git.

## RTL Simulation

The testbenches are in `Hardware/tb/`, with input vectors in `Hardware/tb/frame_input/` and expected outputs in `Hardware/tb/output/`.

Common testbenches include:

- `tb_fpadd.sv`
- `tb_fpmul.sv`
- `tb_fpinvsqrt.sv`
- `tb_core_accel.sv`
- `tb_four_core_wrapper.sv`
- `tb_nbody_mem.sv`
- `tb_nbody_integrator.sv`
- `tb_nbody_control.sv`

Example VCS invocation:

```sh
cd Hardware
vcs -sverilog code/FpAdd.sv tb/tb_fpadd.sv -o simv_fpadd
./simv_fpadd
```

For larger tests, include the RTL modules needed by the selected testbench. Generated `simv_*`, `csrc/`, `*.daidir/`, waveform files, and simulator work directories should remain local build artifacts.

## Golden Models

Python reference models live in `Golden/`.

Generate the control-state golden output used by `tb_nbody_control.sv`:

```sh
python3 Golden/golden_control.py
```

Generate the X/Y stream expected from the Avalon-MM accelerator path:

```sh
python3 Golden/golden_avmm_xy.py
```

Both scripts accept arguments for input frame, output path, body count, gap, and hardware capacity:

```sh
python3 Golden/golden_avmm_xy.py \
  --input Hardware/tb/frame_input/frame0_1024binit200_27bits.txt \
  --output Hardware/tb/output/golden_xy_1024binit200_27bits.txt \
  --n-bodies 1024 \
  --gap 1
```

## Software Build

We need to add additional software to complete our code. Connect the FPGA board to the network, configure the network interface, update package information, and bring everything up-to-date.

```sh
ifup eth0
apt update
apt upgrade -y
apt install -y gcc make libusb-1.0-0-dev usbutils
apt install -y wget
apt clean
```

Download and install `linux-headers-4.19.0.tar.gz`, which includes the Makefile for compiling kernel modules.

```sh
wget https://www.cs.columbia.edu/~sedwards/classes/2025/4840-spring/linux-headers-4.19.0.tar.gz
tar Pzxf linux-headers-4.19.0.tar.gz
ls /usr/src/linux-headers-4.19.0
```

And we should be able to get outputs below
```sh
# ls /usr/src/linux-headers-4.19.0
Documentation  arch   drivers  init   mm            scripts  usr
Kconfig        block  firmware ipc    modules.order security virt
Makefile       certs  fs       kernel net           sound
Module.symvers crypto include  lib    samples       tools
```

Install the kernel module management programs (e.g., insmod, rmmod).

```sh
apt install -y kmod
apt clean
```

Run software build commands from `Software/`.

Build the userspace simulation/display app:

```sh
cd CSEE-4840-N-Body/Software
make
```

Build the Linux kernel modules:

```sh
make modules
```

Load the drivers on the board after the FPGA image and matching device tree are in place:

```sh
insmod accelerator_driver.ko
insmod display_driver.ko
```

The drivers register `/dev/nbody` and `/dev/nbody_display`.

The main app is `nbody_app`:

```sh
./nbody_app <Num Bodies> <Gap>
```

Example:

```sh
./nbody_app 1024 1
```

Keyboard controls:

- `Space`: pause/resume
- `W` / `S`: increase/decrease simulation gap
- `A` / `D`: step backward/forward through stored frames
- `R`: reset
- `Q`: quit

## Direct Avalon-MM Tests

`Software/test/` contains small userspace tests that access the HPS lightweight bridge directly through `/dev/mem`.

```sh
cd Software/test
make
make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
```

To dump one frame of accelerator X/Y output:

```sh
make run-frame \
  NBODY_OFFSET=0x00000 \
  FRAME_INPUT=../../Hardware/tb/frame_input/frame0_1024binit200_27bits.txt \
  FRAME_OUTPUT=hw_xy_1024.txt \
  FRAME_N=1024 \
  FRAME_GAP=1
```

Replace `NBODY_OFFSET` and `VGA_OFFSET` with the offsets assigned in Platform Designer relative to the lightweight bridge base address `0xFF200000`.

## Cleaning Generated Files

```sh
cd Hardware
make clean

cd ../Software
make clean
make modules-clean

cd test
make clean
```

Project-local files such as `.qsys_edit/`, Quartus databases, generated Platform Designer output, simulator executables, kernel objects, and local reference copies are ignored so they can stay on your machine without being committed.

## Notes for Contributors

- Keep generated hardware/software artifacts out of Git.
- Regenerate golden outputs when changing datapath, integration, or control ordering.
- Update `Hardware/soc_system.qsys`, IP wrapper TCL, and software register assumptions together when changing the Avalon-MM memory map.
- Prefer checking hardware changes against both RTL testbenches and the Python golden models before testing on the board.

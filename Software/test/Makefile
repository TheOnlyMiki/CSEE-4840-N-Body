# Makefile for direct userspace Avalon-MM smoke test on DE1-SoC
#
# This is intentionally simpler than the Lab 3 driver Makefile:
# it only builds a userspace program that opens /dev/mem and writes/reads
# the FPGA lightweight bridge directly. No .ko driver is built here.
#
# Usage on DE1-SoC:
#   make
#   make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
#   make run-frame NBODY_OFFSET=0x00000 FRAME_INPUT=frame0_1024binit200_27bits.txt FRAME_OUTPUT=hw_xy_1024.txt
#
# Replace the offsets with your Platform Designer assigned offsets
# relative to the HPS lightweight bridge base 0xFF200000.

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
SMOKE_TARGET := avmm_smoke_test
FRAME_TARGET := avmm_frame_xy_dump
TARGETS      := $(SMOKE_TARGET) $(FRAME_TARGET)

# Override these at the command line:
#   make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
NBODY_OFFSET ?= 0x00000
VGA_OFFSET   ?= 0x01000
FRAME_INPUT  ?= ../../Hardware/tb/frame_input/frame0_1024binit200_27bits.txt
FRAME_OUTPUT ?= hw_xy_1024.txt
FRAME_N      ?= 1024
FRAME_GAP    ?= 1

.PHONY: all run clean help

all: $(TARGETS)

$(SMOKE_TARGET): avmm_smoke_test.c
	$(CC) $(CFLAGS) -o $@ $^

$(FRAME_TARGET): avmm_frame_xy_dump.c
	$(CC) $(CFLAGS) -o $@ $^

run: $(SMOKE_TARGET)
	./$(SMOKE_TARGET) $(NBODY_OFFSET) $(VGA_OFFSET)

run-frame: $(FRAME_TARGET)
	./$(FRAME_TARGET) $(NBODY_OFFSET) $(FRAME_INPUT) $(FRAME_OUTPUT) $(FRAME_N) $(FRAME_GAP)

clean:
	$(RM) $(TARGETS) *.o

help:
	@echo "Build: make"
	@echo "Run:   make run NBODY_OFFSET=0x<nbody_offset> VGA_OFFSET=0x<vga_offset>"
	@echo "Example: make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000"
	@echo "Frame: make run-frame NBODY_OFFSET=0x<nbody_offset> FRAME_INPUT=<frame.txt> FRAME_OUTPUT=<xy.txt> FRAME_N=<n> FRAME_GAP=<gap>"
	@echo "Example: make run-frame NBODY_OFFSET=0x00000 FRAME_INPUT=../../Hardware/tb/frame_input/frame0_1024binit200_27bits.txt FRAME_OUTPUT=hw_xy_1024.txt FRAME_N=1024 FRAME_GAP=1"

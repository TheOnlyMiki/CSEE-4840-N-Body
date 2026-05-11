# Makefile for direct userspace Avalon-MM smoke test on DE1-SoC
#
# This is intentionally simpler than the Lab 3 driver Makefile:
# it only builds a userspace program that opens /dev/mem and writes/reads
# the FPGA lightweight bridge directly. No .ko driver is built here.
#
# Usage on DE1-SoC:
#   make
#   make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
#
# Replace the offsets with your Platform Designer assigned offsets
# relative to the HPS lightweight bridge base 0xFF200000.

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
TARGET  := avmm_smoke_test
SRCS    := avmm_smoke_test.c

# Override these at the command line:
#   make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000
NBODY_OFFSET ?= 0x00000
VGA_OFFSET   ?= 0x01000

.PHONY: all run clean help

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

run: $(TARGET)
	./$(TARGET) $(NBODY_OFFSET) $(VGA_OFFSET)

clean:
	$(RM) $(TARGET) *.o

help:
	@echo "Build: make"
	@echo "Run:   make run NBODY_OFFSET=0x<nbody_offset> VGA_OFFSET=0x<vga_offset>"
	@echo "Example: make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000"

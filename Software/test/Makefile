CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CPPFLAGS ?=
LDFLAGS ?=
LDLIBS ?=
SUDO ?= sudo

TARGET := avmm_smoke_test
SRC := avmm_smoke_test.c

NBODY_OFFSET ?= 0x00000
VGA_OFFSET ?= 0x01000

.PHONY: all run run-nosudo clean help

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(SUDO) ./$(TARGET) $(NBODY_OFFSET) $(VGA_OFFSET)

run-nosudo: $(TARGET)
	./$(TARGET) $(NBODY_OFFSET) $(VGA_OFFSET)

clean:
	rm -f $(TARGET)

help:
	@printf 'Targets:\n'
	@printf '  make              Build %s\n' '$(TARGET)'
	@printf '  make run          Build and run with sudo\n'
	@printf '  make run-nosudo   Build and run without sudo\n'
	@printf '  make clean        Remove built binary\n\n'
	@printf 'Offset overrides:\n'
	@printf '  make run NBODY_OFFSET=0x00000 VGA_OFFSET=0x01000\n'

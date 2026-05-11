/*
 * avmm_smoke_test.c
 *
 * Minimal Linux userspace smoke test for DE1-SoC HPS -> FPGA lightweight
 * Avalon-MM peripherals:
 *   - nbody_accel_avmm
 *   - vga_bitmap_avmm
 *
 * This does NOT prove accelerator math correctness yet. It only checks that
 * software can reach the expected Avalon address windows and exercise the
 * simplest control/read paths.
 *
 * Build on the board:
 *   gcc -O2 -Wall -Wextra -o avmm_smoke_test avmm_smoke_test.c
 *
 * Run as root / with permission to open /dev/mem:
 *   ./avmm_smoke_test <nbody_offset_hex> <vga_offset_hex>
 *
 * Example only; replace offsets with the addresses assigned in Platform Designer
 * relative to the HPS lightweight bridge:
 *   ./avmm_smoke_test 0x00000 0x01000
 */

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define LW_BRIDGE_BASE 0xFF200000u
#define LW_BRIDGE_SPAN 0x00200000u

/* nbody_accel_avmm register word indices */
enum {
    NB_GO       = 0x00,
    NB_N_BODIES = 0x01,
    NB_GAP      = 0x02,
    NB_X_IN     = 0x03,
    NB_Y_IN     = 0x04,
    NB_M_IN     = 0x05,
    NB_VX_IN    = 0x06,
    NB_VY_IN    = 0x07,
    NB_DONE     = 0x08,
    NB_READ     = 0x09,
    NB_OUT_X    = 0x0A,
    NB_OUT_Y    = 0x0B,
};

/* vga_bitmap_avmm constants */
#define VGA_WIDTH         640u
#define VGA_HEIGHT        480u
#define VGA_WORDS_PER_ROW 20u
#define VGA_FB_WORDS      (VGA_HEIGHT * VGA_WORDS_PER_ROW)

/* 27-bit custom float S1E8M18 helpers for exact simple values. */
#define FP27_ZERO 0x00000000u
#define FP27_ONE  0x01FC0000u  /* sign=0, exp=127, mant=0 */
#define FP27_TWO  0x02000000u  /* sign=0, exp=128, mant=0 */

static inline void mmio_write(volatile uint32_t *base, uint32_t word_index, uint32_t value) {
    base[word_index] = value;
    __sync_synchronize();
}

static inline uint32_t mmio_read(volatile uint32_t *base, uint32_t word_index) {
    uint32_t value = base[word_index];
    __sync_synchronize();
    return value;
}

static uint32_t parse_hex_arg(const char *s, const char *name) {
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0' || value > 0xFFFFFFFFul) {
        fprintf(stderr, "Bad %s: %s\n", name, s);
        exit(2);
    }
    if ((value & 0x3u) != 0) {
        fprintf(stderr, "Warning: %s is not 4-byte aligned: 0x%08lx\n", name, value);
    }
    return (uint32_t)value;
}

static bool poll_done(volatile uint32_t *nb, unsigned max_iters) {
    for (unsigned i = 0; i < max_iters; ++i) {
        if (mmio_read(nb, NB_DONE) & 0x1u) {
            return true;
        }
    }
    return false;
}

static void vga_clear(volatile uint32_t *vga, uint32_t value) {
    for (uint32_t i = 0; i < VGA_FB_WORDS; ++i) {
        mmio_write(vga, i, value);
    }
}

static void vga_draw_test_pattern(volatile uint32_t *vga) {
    /* Clear screen. */
    vga_clear(vga, 0x00000000u);

    /* Draw a simple border plus a diagonal-ish line. */
    for (uint32_t y = 0; y < VGA_HEIGHT; ++y) {
        for (uint32_t xword = 0; xword < VGA_WORDS_PER_ROW; ++xword) {
            uint32_t word = 0;
            uint32_t x0 = xword * 32u;
            for (uint32_t b = 0; b < 32u; ++b) {
                uint32_t x = x0 + b;
                bool border = (x == 0u || x == VGA_WIDTH - 1u || y == 0u || y == VGA_HEIGHT - 1u);
                bool diag = (x < VGA_WIDTH && (x / 2u) == y);
                if (border || diag) {
                    word |= (1u << b); /* LSB-first, matching hardware comment. */
                }
            }
            mmio_write(vga, y * VGA_WORDS_PER_ROW + xword, word);
        }
    }
}

static int test_nbody_zero_body_done(volatile uint32_t *nb) {
    printf("[nbody] zero-body GO/DONE/READ smoke test...\n");

    mmio_write(nb, NB_READ, 0u);
    mmio_write(nb, NB_N_BODIES, 0u);
    mmio_write(nb, NB_GAP, 1u);
    mmio_write(nb, NB_GO, 1u);

    if (!poll_done(nb, 1000000u)) {
        fprintf(stderr, "[FAIL] DONE did not go high for N_BODIES=0.\n");
        return 1;
    }
    printf("[pass] DONE went high.\n");

    mmio_write(nb, NB_READ, 0u);
    for (unsigned i = 0; i < 1000u; ++i) {
        if ((mmio_read(nb, NB_DONE) & 0x1u) == 0u) {
            printf("[pass] DONE cleared after READ=0.\n");
            return 0;
        }
    }

    fprintf(stderr, "[FAIL] DONE did not clear after READ=0.\n");
    return 1;
}

static int test_nbody_one_body_output(volatile uint32_t *nb) {
    printf("[nbody] one-body input/output smoke test...\n");

    /* Load one body: x=1.0, y=2.0, m=1.0, vx=0, vy=0. */
    mmio_write(nb, NB_READ, 0u);
    mmio_write(nb, NB_N_BODIES, 1u);
    mmio_write(nb, NB_GAP, 1u);
    mmio_write(nb, NB_X_IN, FP27_ONE);
    mmio_write(nb, NB_Y_IN, FP27_TWO);
    mmio_write(nb, NB_M_IN, FP27_ONE);
    mmio_write(nb, NB_VX_IN, FP27_ZERO);
    mmio_write(nb, NB_VY_IN, FP27_ZERO); /* commits body 0 */
    mmio_write(nb, NB_GO, 1u);

    if (!poll_done(nb, 10000000u)) {
        fprintf(stderr, "[FAIL] DONE did not go high for N_BODIES=1.\n");
        return 1;
    }

    uint32_t out_x = mmio_read(nb, NB_OUT_X) & 0x07FFFFFFu;
    uint32_t out_y = mmio_read(nb, NB_OUT_Y) & 0x07FFFFFFu; /* increments output pointer */

    printf("[info] OUT_X = 0x%08" PRIx32 ", OUT_Y = 0x%08" PRIx32 "\n", out_x, out_y);
    printf("[info] Expected for current N=1 zero-velocity smoke case is usually x=0x%08x, y=0x%08x.\n",
           FP27_ONE, FP27_TWO);

    if (out_x == FP27_ONE && out_y == FP27_TWO) {
        printf("[pass] one-body output matches input position.\n");
    } else {
        printf("[warn] one-body output did not exactly match. This may indicate either a real issue or that the integrator/first-step path changed the value.\n");
    }

    mmio_write(nb, NB_READ, 0u);
    return 0;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr,
                "Usage: %s <nbody_offset_hex> <vga_offset_hex>\n"
                "Example: %s 0x00000 0x01000\n",
                argv[0], argv[0]);
        return 2;
    }

    uint32_t nbody_offset = parse_hex_arg(argv[1], "nbody offset");
    uint32_t vga_offset   = parse_hex_arg(argv[2], "vga offset");

    if (nbody_offset >= LW_BRIDGE_SPAN || vga_offset >= LW_BRIDGE_SPAN) {
        fprintf(stderr, "Offsets must be inside LW bridge span 0x%08x.\n", LW_BRIDGE_SPAN);
        return 2;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    void *map = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (map == MAP_FAILED) {
        perror("mmap lightweight bridge");
        close(fd);
        return 1;
    }

    volatile uint32_t *nbody = (volatile uint32_t *)((volatile uint8_t *)map + nbody_offset);
    volatile uint32_t *vga   = (volatile uint32_t *)((volatile uint8_t *)map + vga_offset);

    printf("LW bridge base: 0x%08x\n", LW_BRIDGE_BASE);
    printf("nbody offset  : 0x%08" PRIx32 "\n", nbody_offset);
    printf("vga offset    : 0x%08" PRIx32 "\n", vga_offset);

    int failures = 0;

    /* VGA readback is intentionally always zero in current RTL. */
    printf("[vga] readback check: word0=0x%08" PRIx32 " word123=0x%08" PRIx32 " (current RTL should read 0)\n",
           mmio_read(vga, 0), mmio_read(vga, 123));
    printf("[vga] drawing border + diagonal test pattern...\n");
    vga_draw_test_pattern(vga);
    printf("[vga] done. Check monitor for white border and diagonal on black background.\n");

    failures += test_nbody_zero_body_done(nbody);
    failures += test_nbody_one_body_output(nbody);

    munmap(map, LW_BRIDGE_SPAN);
    close(fd);

    if (failures == 0) {
        printf("All smoke tests completed.\n");
        return 0;
    }

    fprintf(stderr, "%d smoke test(s) failed.\n", failures);
    return 1;
}

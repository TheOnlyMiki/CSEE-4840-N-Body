#define _DEFAULT_SOURCE

/*
 * avmm_frame_xy_dump.c
 *
 * Board-side N-body Avalon-MM test that loads a frame-input txt file, runs one
 * accelerator pass, and writes the resulting X/Y positions to a txt file for
 * comparison against a golden output.
 *
 * Input format matches the txt files in Hardware/tb/frame_input:
 *   # i  px  py  vx  vy  m   (S1E8M18 hex)
 *      0  20CE1A0  2156136  0000000  0000000  1FA8587
 *
 * Build on the board:
 *   gcc -O2 -Wall -Wextra -std=c11 -o avmm_frame_xy_dump avmm_frame_xy_dump.c
 *
 * Run as root / with permission to open /dev/mem:
 *   ./avmm_frame_xy_dump <nbody_offset_hex> <input_frame.txt> <output_xy.txt> [n_bodies] [gap]
 *
 * Example:
 *   ./avmm_frame_xy_dump 0x00000 frame0_1024binit200_27bits.txt hw_xy_1024.txt 1024 1
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

#define NBODY_MAX_BODIES 1024u
#define NBODY_DATA_MASK  0x07FFFFFFu

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

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t vx;
    uint32_t vy;
    uint32_t m;
    bool present;
} frame_body_t;

static inline void mmio_write(volatile uint32_t *base, uint32_t word_index, uint32_t value)
{
    base[word_index] = value;
    __sync_synchronize();
}

static inline uint32_t mmio_read(volatile uint32_t *base, uint32_t word_index)
{
    uint32_t value = base[word_index];
    __sync_synchronize();
    return value;
}

static uint32_t parse_u32_arg(const char *s, const char *name)
{
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0' || value > 0xFFFFFFFFul) {
        fprintf(stderr, "Bad %s: %s\n", name, s);
        exit(2);
    }
    return (uint32_t)value;
}

static uint32_t parse_count_arg(const char *s, const char *name)
{
    uint32_t value = parse_u32_arg(s, name);
    if (value == 0u || value > NBODY_MAX_BODIES) {
        fprintf(stderr, "%s must be in 1..%u, got %" PRIu32 "\n",
                name, NBODY_MAX_BODIES, value);
        exit(2);
    }
    return value;
}

static uint32_t parse_gap_arg(const char *s)
{
    uint32_t value = parse_u32_arg(s, "gap");
    if (value == 0u) {
        fprintf(stderr, "gap must be nonzero\n");
        exit(2);
    }
    return value;
}

static uint32_t parse_offset_arg(const char *s)
{
    uint32_t value = parse_u32_arg(s, "nbody offset");
    if ((value & 0x3u) != 0u) {
        fprintf(stderr, "Warning: nbody offset is not 4-byte aligned: 0x%08" PRIx32 "\n",
                value);
    }
    if (value >= LW_BRIDGE_SPAN) {
        fprintf(stderr, "nbody offset must be inside LW bridge span 0x%08x\n",
                LW_BRIDGE_SPAN);
        exit(2);
    }
    return value;
}

static uint32_t infer_count(const frame_body_t bodies[NBODY_MAX_BODIES])
{
    uint32_t count = 0;

    for (uint32_t i = 0; i < NBODY_MAX_BODIES; ++i) {
        if (bodies[i].present) {
            count = i + 1u;
        }
    }

    return count;
}

static int read_frame_file(const char *path, frame_body_t bodies[NBODY_MAX_BODIES],
                           uint32_t *inferred_count)
{
    FILE *fp;
    char line[512];
    unsigned line_no = 0;

    memset(bodies, 0, sizeof(frame_body_t) * NBODY_MAX_BODIES);

    fp = fopen(path, "r");
    if (!fp) {
        perror(path);
        return 1;
    }

    while (fgets(line, sizeof(line), fp)) {
        int idx;
        unsigned px;
        unsigned py;
        unsigned vx;
        unsigned vy;
        unsigned m;
        char tail;
        int got;
        char *p = line;

        line_no++;
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0' || *p == '\n' || *p == '#') {
            continue;
        }

        got = sscanf(p, "%d %x %x %x %x %x %c", &idx, &px, &py, &vx, &vy, &m, &tail);
        if (got != 6) {
            fprintf(stderr, "%s:%u: expected six columns: i px py vx vy m\n",
                    path, line_no);
            fclose(fp);
            return 1;
        }
        if (idx < 0 || idx >= (int)NBODY_MAX_BODIES) {
            fprintf(stderr, "%s:%u: body index %d outside 0..%u\n",
                    path, line_no, idx, NBODY_MAX_BODIES - 1u);
            fclose(fp);
            return 1;
        }
        if ((px | py | vx | vy | m) & ~NBODY_DATA_MASK) {
            fprintf(stderr, "%s:%u: S1E8M18 fields must fit in 27 bits\n",
                    path, line_no);
            fclose(fp);
            return 1;
        }
        if (bodies[idx].present) {
            fprintf(stderr, "%s:%u: duplicate body index %d\n", path, line_no, idx);
            fclose(fp);
            return 1;
        }

        bodies[idx].x = px;
        bodies[idx].y = py;
        bodies[idx].vx = vx;
        bodies[idx].vy = vy;
        bodies[idx].m = m;
        bodies[idx].present = true;
    }

    if (ferror(fp)) {
        perror(path);
        fclose(fp);
        return 1;
    }

    fclose(fp);
    *inferred_count = infer_count(bodies);
    if (*inferred_count == 0u) {
        fprintf(stderr, "%s: no body rows found\n", path);
        return 1;
    }

    return 0;
}

static int validate_frame_prefix(const frame_body_t bodies[NBODY_MAX_BODIES],
                                 uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i) {
        if (!bodies[i].present) {
            fprintf(stderr, "input frame is missing required body index %" PRIu32 "\n", i);
            return 1;
        }
    }
    return 0;
}

static void load_frame(volatile uint32_t *nb, const frame_body_t bodies[NBODY_MAX_BODIES],
                       uint32_t count, uint32_t gap)
{
    mmio_write(nb, NB_READ, 0u);
    mmio_write(nb, NB_N_BODIES, count);
    mmio_write(nb, NB_GAP, gap);

    for (uint32_t i = 0; i < count; ++i) {
        mmio_write(nb, NB_X_IN, bodies[i].x);
        mmio_write(nb, NB_Y_IN, bodies[i].y);
        mmio_write(nb, NB_M_IN, bodies[i].m);
        mmio_write(nb, NB_VX_IN, bodies[i].vx);
        mmio_write(nb, NB_VY_IN, bodies[i].vy);
    }
}

static bool poll_done(volatile uint32_t *nb, uint32_t max_iters)
{
    for (uint32_t i = 0; i < max_iters; ++i) {
        if (mmio_read(nb, NB_DONE) & 0x1u) {
            return true;
        }
        if ((i & 0x3fffu) == 0u) {
            usleep(1000);
        }
    }
    return false;
}

static int run_and_write_xy(volatile uint32_t *nb, const char *out_path,
                            const char *input_path, uint32_t count, uint32_t gap)
{
    FILE *out;

    mmio_write(nb, NB_GO, 1u);
    if (!poll_done(nb, 20000000u)) {
        fprintf(stderr, "[FAIL] DONE did not go high for count=%" PRIu32 ", gap=%" PRIu32 "\n",
                count, gap);
        return 1;
    }

    out = fopen(out_path, "w");
    if (!out) {
        perror(out_path);
        return 1;
    }

    fprintf(out, "# source %s\n", input_path);
    fprintf(out, "# n_bodies %" PRIu32 "\n", count);
    fprintf(out, "# gap %" PRIu32 "\n", gap);
    fprintf(out, "# i x y (S1E8M18 hex)\n");

    mmio_write(nb, NB_READ, 1u);
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t x = mmio_read(nb, NB_OUT_X) & NBODY_DATA_MASK;
        uint32_t y = mmio_read(nb, NB_OUT_Y) & NBODY_DATA_MASK;
        fprintf(out, "%4" PRIu32 "  %07" PRIX32 "  %07" PRIX32 "\n", i, x, y);
    }
    mmio_write(nb, NB_READ, 0u);

    if (fclose(out) != 0) {
        perror(out_path);
        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    frame_body_t bodies[NBODY_MAX_BODIES];
    uint32_t nbody_offset;
    uint32_t inferred_count;
    uint32_t count;
    uint32_t gap;
    int fd;
    void *map;
    volatile uint32_t *nbody;
    int ret;

    if (argc < 4 || argc > 6) {
        fprintf(stderr,
                "Usage: %s <nbody_offset_hex> <input_frame.txt> <output_xy.txt> [n_bodies] [gap]\n"
                "Example: %s 0x00000 frame0_1024binit200_27bits.txt hw_xy_1024.txt 1024 1\n",
                argv[0], argv[0]);
        return 2;
    }

    nbody_offset = parse_offset_arg(argv[1]);
    gap = (argc >= 6) ? parse_gap_arg(argv[5]) : 1u;

    if (read_frame_file(argv[2], bodies, &inferred_count) != 0) {
        return 1;
    }

    count = (argc >= 5) ? parse_count_arg(argv[4], "n_bodies") : inferred_count;
    if (count > inferred_count) {
        fprintf(stderr,
                "requested n_bodies=%" PRIu32 " but frame only contains rows through index %" PRIu32 "\n",
                count, inferred_count - 1u);
        return 2;
    }
    if (validate_frame_prefix(bodies, count) != 0) {
        return 1;
    }

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    map = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (map == MAP_FAILED) {
        perror("mmap lightweight bridge");
        close(fd);
        return 1;
    }

    nbody = (volatile uint32_t *)((volatile uint8_t *)map + nbody_offset);

    printf("LW bridge base: 0x%08x\n", LW_BRIDGE_BASE);
    printf("nbody offset  : 0x%08" PRIx32 "\n", nbody_offset);
    printf("input frame   : %s\n", argv[2]);
    printf("output xy     : %s\n", argv[3]);
    printf("n_bodies      : %" PRIu32 "\n", count);
    printf("gap           : %" PRIu32 "\n", gap);
    printf("[nbody] loading frame...\n");

    load_frame(nbody, bodies, count, gap);
    printf("[nbody] running accelerator and dumping X/Y results...\n");
    ret = run_and_write_xy(nbody, argv[3], argv[2], count, gap);

    munmap(map, LW_BRIDGE_SPAN);
    close(fd);

    if (ret == 0) {
        printf("[pass] wrote %s\n", argv[3]);
    }
    return ret;
}

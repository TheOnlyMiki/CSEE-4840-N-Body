#include <ctype.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "display.h"
#include "nbody.h"

static int display_fd = -1;
static uint32_t framebuffer[DISPLAY_WORDS];
static uint8_t body_masks[NUM_BODY_RADII][9][9];

static const int BODY_RADII[NUM_BODY_RADII] = { 1, 2, 3, 4 };

/*
 * Packed display format from vga_bitmap_avmm.sv:
 * word = y * 20 + x / 32, bit = x % 32, one LSB-first bit per pixel.
 */
static inline void set_pixel(int x, int y, int on)
{
    int word;
    int bit;

    if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT)
        return;

    word = y * DISPLAY_WORDS_PER_ROW + x / 32;
    bit = x % 32;

    if (on)
        framebuffer[word] |= (1u << bit);
    else
        framebuffer[word] &= ~(1u << bit);
}

static uint8_t glyph_row(char c, int row)
{
    static const uint8_t blank[7] = { 0, 0, 0, 0, 0, 0, 0 };
    static const uint8_t glyphs[][7] = {
        ['0'] = { 0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e },
        ['1'] = { 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e },
        ['2'] = { 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f },
        ['3'] = { 0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e },
        ['4'] = { 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 },
        ['5'] = { 0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e },
        ['6'] = { 0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e },
        ['7'] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
        ['8'] = { 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e },
        ['9'] = { 0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e },
        ['A'] = { 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 },
        ['B'] = { 0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e },
        ['C'] = { 0x0f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0f },
        ['D'] = { 0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e },
        ['E'] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f },
        ['F'] = { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
        ['G'] = { 0x0f, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0f },
        ['H'] = { 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 },
        ['I'] = { 0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e },
        ['J'] = { 0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c },
        ['K'] = { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
        ['L'] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f },
        ['M'] = { 0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11 },
        ['N'] = { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 },
        ['O'] = { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },
        ['P'] = { 0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10 },
        ['Q'] = { 0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d },
        ['R'] = { 0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11 },
        ['S'] = { 0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e },
        ['T'] = { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
        ['U'] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e },
        ['V'] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04 },
        ['W'] = { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a },
        ['X'] = { 0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11 },
        ['Y'] = { 0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04 },
        ['Z'] = { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f },
        [':'] = { 0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00 },
        ['/'] = { 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10 },
        ['['] = { 0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e },
        [']'] = { 0x0e, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0e },
        ['|'] = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
        ['-'] = { 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00 },
        [' '] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    };
    const uint8_t *g;

    if (c >= 'a' && c <= 'z')
        c = (char)toupper((unsigned char)c);

    if ((unsigned char)c >= sizeof(glyphs) / sizeof(glyphs[0]))
        g = blank;
    else
        g = glyphs[(unsigned char)c];

    return g[row];
}

int display_open_device(const char *path)
{
    display_fd = open(path, O_RDWR);
    return display_fd;
}

void display_close_device(void)
{
    if (display_fd >= 0)
        close(display_fd);
    display_fd = -1;
}

void display_clear(void)
{
    memset(framebuffer, 0, sizeof(framebuffer));
}

void display_clear_bodyscreen(void)
{
    memset(framebuffer, 0, DISPLAY_WORDS_PER_ROW * BODY_DISPLAY_H * sizeof(uint32_t));
}

void display_clear_ui(void)
{
    memset(&framebuffer[DISPLAY_WORDS_PER_ROW * BODY_DISPLAY_H], 0,
           DISPLAY_WORDS_PER_ROW * (DISPLAY_HEIGHT - BODY_DISPLAY_H) * sizeof(uint32_t));
}

void display_init_bodyshape(void)
{
    int i;

    memset(body_masks, 0, sizeof(body_masks));
    for (i = 0; i < NUM_BODY_RADII; i++) {
        int r = BODY_RADII[i];
        int x;
        int y;

        for (y = -r; y <= r; y++)
            for (x = -r; x <= r; x++)
                if (x * x + y * y < r * r + r)
                    body_masks[i][y + r][x + r] = 1;
    }
}

void display_draw_body(int cx, int cy, int radius_idx)
{
    int r;
    int x;
    int y;

    if (radius_idx < 0)
        radius_idx = 0;
    if (radius_idx >= NUM_BODY_RADII)
        radius_idx = NUM_BODY_RADII - 1;

    r = BODY_RADII[radius_idx];
    for (y = -r; y <= r; y++) {
        int sy = cy + y;
        if (sy < 0 || sy >= BODY_DISPLAY_H)
            continue;
        for (x = -r; x <= r; x++)
            if (body_masks[radius_idx][y + r][x + r])
                set_pixel(cx + x, sy, 1);
    }
}

void display_putchar(char c, int row, int col)
{
    int y;

    if (row < 0 || row >= TEXT_ROWS || col < 0 || col >= TEXT_COLS)
        return;

    for (y = 0; y < FONT_HEIGHT; y++) {
        uint8_t bits = 0;
        int x;

        if (y >= 2 && y < 16)
            bits = glyph_row(c, (y - 2) / 2);

        for (x = 0; x < FONT_WIDTH; x++) {
            int on = (x >= 1 && x <= 5) && (bits & (1u << (5 - x)));
            set_pixel(col * FONT_WIDTH + x, row * FONT_HEIGHT + y, on);
        }
    }
}

void display_puts(const char *s, int row, int col)
{
    while (*s && col < TEXT_COLS)
        display_putchar(*s++, row, col++);
}

int display_present(void)
{
    if (display_fd < 0)
        return -1;
    return ioctl(display_fd, DISPLAY_WRITE_FRAME, framebuffer);
}

void *display_thread(void *arg)
{
    body_pos_t *render_positions = malloc(MAX_BODIES * sizeof(*render_positions));

    (void)arg;
    if (!render_positions)
        return NULL;

    if (display_open_device("/dev/nbody_display") < 0)
        perror("open /dev/nbody_display");

    display_init_bodyshape();

    while (running) {
        int local_num;
        int local_gap;
        int local_count;
        int local_view;
        int local_paused;
        int slot;
        char line1[TEXT_COLS + 1];
        int i;

        pthread_mutex_lock(&state_mutex);
        local_num = num_bodies;
        local_gap = current_gap;
        local_count = h_count;
        local_view = view_idx;
        local_paused = is_paused;

        if (local_count > 0) {
            if (local_count < MAX_HISTORY)
                slot = local_view;
            else
                slot = (h_head + local_view) & (MAX_HISTORY - 1);
            memcpy(render_positions, history[slot],
                   local_num * sizeof(*render_positions));
        }
        pthread_mutex_unlock(&state_mutex);

        display_clear();
        if (local_count > 0) {
            for (i = 0; i < local_num; i++) {
                int x = world_to_screen_x(render_positions[i].x);
                int y = world_to_screen_y(render_positions[i].y);
                display_draw_body(x, y, (int)static_masses[i]);
            }
        }

        snprintf(line1, sizeof(line1), "Bodies: %4d/%d | Gap: %2d/%d | Frame: %d/%d | Status: %s",
                 local_num, MAX_BODIES, local_gap, 10, local_view + 1, local_count,
                 local_paused ? "PAUSED" : "RUNNING");
        display_puts(line1, UI_START_ROW, 0);
        display_puts("[SPACE] Play/Pause | [W/S] Gap | [A/D] Frame | [R] Reset | [Q] Quit",
                     UI_START_ROW + 1, 0);
        display_present();
        usleep(33333);
    }

    if (display_fd >= 0) {
        display_clear();
        display_present();
    }
    display_close_device();
    free(render_positions);
    return NULL;
}

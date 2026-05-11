#ifndef _DISPLAY_H
#define _DISPLAY_H

#include "display_ioctl.h"

#define FONT_WIDTH 8
#define FONT_HEIGHT 16
#define TEXT_COLS 80
#define TEXT_ROWS 30
#define UI_START_ROW 28
#define BODY_DISPLAY_H 448
#define NUM_BODY_RADII 4

int display_open_device(const char *path);
void display_close_device(void);
void display_clear(void);
void display_clear_bodyscreen(void);
void display_clear_ui(void);
void display_init_bodyshape(void);
void display_draw_body(int cx, int cy, int radius_idx);
void display_putchar(char c, int row, int col);
void display_puts(const char *s, int row, int col);
int display_present(void);
void *display_thread(void *arg);

#endif

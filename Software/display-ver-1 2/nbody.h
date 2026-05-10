#ifndef _NBODY_H
#define _NBODY_H

#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include "nbody_ioctl.h"

#define MAX_BODIES 1024
#define MAX_HISTORY 256

// 屏幕分辨率为 1024 x 768，底部两行文字(16像素高)占用 768 - 32 = 736 之前的区域
#define BODY_DISPLAY_W 1024
#define BODY_DISPLAY_H 736   

// 底部 UI 起始行号, 屏幕一共为 128 列 x 48 行 的空间
#define UI_START_ROW 46   

/* 全局变量声明 */
extern int num_bodies;
extern int current_gap;
extern int is_paused;
extern int running;

extern uint32_t static_masses[MAX_BODIES];
extern body_pos_t *history[MAX_HISTORY];
extern int h_head;
extern int h_count;
extern int view_idx;

extern int nbody_fd;
extern struct libusb_device_handle *keyboard;
extern uint8_t endpoint_address;

/* 核心功能函数声明 */
int get_radius_idx(uint32_t mass);
void reset_system(void);
void draw_body(void);
void draw_ui(void);
void *keyboard_handler(void *arg);

#endif

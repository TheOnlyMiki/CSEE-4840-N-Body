#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "nbody.h"
#include "framebuffer.h"
#include "usbkeyboard.h"

/* 定义在 nbody.h 声明的全局变量 */
int num_bodies = 0;
int current_gap = 1;
int is_paused = 0;
int running = 1;
int vga_fd = -1;

uint32_t static_masses[MAX_BODIES];
body_pos_t *history[MAX_HISTORY];
int h_head = 0;
int h_count = 0;
int view_idx = 0;

int nbody_fd = -1;
struct libusb_device_handle *keyboard = NULL;
uint8_t endpoint_address;

/* 根据质量获取对应的半径掩码索引 (0到4, 对应半径1到5) */
int get_radius_idx(uint32_t mass) {
    if (mass <= 34) return 0;
    if (mass <= 67) return 1;
    return 2;
}

/* 初始化或重置系统状态 */
void reset_system(void) {
    body_init_t *init_data = malloc(sizeof(body_init_t) * num_bodies);
    if (!init_data) {
        fprintf(stderr, "Failed to allocate init_data memory.\n");
        return;
    }

    for (int i = 0; i < num_bodies; i++) {
        // 1. 随机生成初始状态
        init_data[i].x = rand() % BODY_DISPLAY_W;
        init_data[i].y = rand() % BODY_DISPLAY_H;
        init_data[i].vx = (rand() % 10) + 1;
        init_data[i].vy = (rand() % 10) + 1;
        if (rand() % 2 == 0) init_data[i].vx = -init_data[i].vx;
        if (rand() % 2 == 0) init_data[i].vy = -init_data[i].vy;
        init_data[i].mass = (rand() % 100) + 1;

        // 2. 软件侧仅保存不变量(mass)
        static_masses[i] = get_radius_idx(init_data[i].mass);
        
        // 3. 记录第 0 帧坐标进入历史缓冲区
        history[0][i].x = init_data[i].x;
        history[0][i].y = init_data[i].y;
    }

    // 4. 将初始数据一次性下发给硬件 (Avalon Bus)
    // 4. 将初始数据一次性下发给硬件 (Avalon Bus)
    if (nbody_fd != -1) {
        ioctl(nbody_fd, NBODY_WRITE_INIT_DATA, init_data);
    }

    free(init_data);

    // 5. 通知VGA driver球的数量和每个球的半径
    if (vga_fd != -1) {
        uint32_t n = num_bodies;
        ioctl(vga_fd, VGA_SET_NUM_BODIES, &n);       // 告诉driver有多少个球
        ioctl(vga_fd, VGA_SET_RADIUS, static_masses); // 告诉driver每个球的半径
    }

    // 5. 初始化历史记录指针
    h_head = 1;
    h_count = 1;
    view_idx = 0;
}

/* 刷新粒子的画面 */
/*void draw_body(void) {
    // 清除原有的粒子区域
    fbclear_bodyscreen(); 

    body_pos_t *render_frame = history[view_idx];
        
    for (int i = 0; i < num_bodies; i++) {
        // 所有天体画白色 (255, 255, 255)
        fbdrawbody(render_frame[i].x, render_frame[i].y, static_masses[i], 255, 255, 255);
    }
}*/
void draw_body(void) {
    if (vga_fd != -1) {
        ioctl(vga_fd, VGA_DRAW_FRAME, history[view_idx]);
    }
}
    

/* 刷新底部两行 UI */
void draw_ui(void) {
    char ui_line1[128];
    char ui_line2[128];

    // 清除原有的 UI 区域
    //fbclear_ui();

    sprintf(ui_line1, "Bodies: %4d/%d | Gap: %2d/10 | Frame (Max 256): %3d/%-3d | Status: %s", 
            num_bodies, MAX_BODIES, current_gap, 
            view_idx + 1, h_count, 
            is_paused ? "PAUSED " : "RUNNING"
        );
    
    sprintf(ui_line2, "[SPACE] Play/Pause | [W/S] Gap Increase/Decrease | [A/D] Frame Backward/Forward | [R] Reset | [Q] Exit");

    fbputs(ui_line1, UI_START_ROW, 0);
    fbputs(ui_line2, UI_START_ROW + 1, 0);
}

/* 键盘事件监听线程 */
void *keyboard_handler(void *arg) {
    struct usb_keyboard_packet packet;
    int transferred;
    uint8_t last_keycode = 0;
    uint8_t keycode;
    uint8_t last_packet[8] = {0};
    
    while (running) {
        libusb_interrupt_transfer(keyboard, endpoint_address,
                                  (unsigned char *)&packet, sizeof(packet),
                                  &transferred, 0);
        if (transferred == sizeof(packet)) {
            // Only when the bag is different from the one last time is it considered a new key pulse
            if (memcmp(&packet, last_packet, sizeof(packet)) == 0) { continue; }

            memcpy(last_packet, &packet, sizeof(packet));

            keycode = packet.keycode[0];
            
            // Only process on key-down (ignore release or repeat of same key)
            if (keycode == last_keycode) { continue; }

            last_keycode = keycode;
            
            if (keycode != 0) {
                // 'SPACE' - Pause/Play
                if (keycode == 0x2C) { 
                    if (view_idx + 1 == h_count) is_paused = !is_paused;
                }
                // 'W' - Gap increse
                else if (keycode == 0x1A) { 
                    if (current_gap < 10) {
                        current_gap++;
                        nbody_config_t config = {num_bodies, current_gap};
                        ioctl(nbody_fd, NBODY_WRITE_CONFIG, &config);
                    }
                } 
                // 'S' - Gap decrese
                else if (keycode == 0x16) {
                    if (current_gap > 1) {
                        current_gap--;
                        nbody_config_t config = {num_bodies, current_gap};
                        ioctl(nbody_fd, NBODY_WRITE_CONFIG, &config);
                    }
                } 
                // 'A' - Frame/History backward 
                else if (keycode == 0x04) { 
                    is_paused = 1; 
                    if (view_idx > 0) { 
                        view_idx--;
                        draw_body();
                    }
                } 
                // 'D' - Frame/History forward
                else if (keycode == 0x07) { 
                    is_paused = 1;
                    if (view_idx < h_count - 1) {
                        view_idx++;
                        draw_body();
                    }
                } 
                // 'R' - Reset
                else if (keycode == 0x15) {
                    is_paused = 1;
                    reset_system();
                    draw_body();
                } 
                // 'Q' - Quit
                else if (keycode == 0x14) { 
                    running = 0;
                }
                
                // 按键发生变化后，立刻更新 UI
                draw_ui(); 
            }
        }
    }

    return NULL;
}

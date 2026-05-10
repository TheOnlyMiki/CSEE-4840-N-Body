#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>

#include "framebuffer.h"
#include "usbkeyboard.h"
#include "nbody.h"

int main(int argc, char **argv) {
    // 1. 启动参数校验与 MAX_BODIES 限制
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <Num Bodies>(1-1024) <Gap>(1-10)\n", argv[0]);
        exit(1);
    }
    
    num_bodies = atoi(argv[1]);
    current_gap = atoi(argv[2]);
    
    if (num_bodies < 1 || num_bodies > MAX_BODIES) {
        fprintf(stderr, "Error: <Num Bodies> must be between 1 and %d.\n", MAX_BODIES);
        exit(1);
    }
    if (current_gap < 1 || current_gap > 10) {
        fprintf(stderr, "Error: <Gap> must be between 1 and 10.\n");
        exit(1);
    }

    // 2. 初始化 Framebuffer
    if (fbopen() != 0) {
        fprintf(stderr, "Error: Could not open framebuffer.\n");
        exit(1);
    }

    // 3. 初始化 USB 键盘
    if ((keyboard = openkeyboard(&endpoint_address)) == NULL) {
        fprintf(stderr, "Error: Did not find a keyboard.\n");
        exit(1);
    }
    
    // 4. 初始化 硬件加速器 设备
    if ((nbody_fd = open("/dev/nbody", O_RDWR)) == -1) {
        fprintf(stderr, "Warning: Could not open /dev/nbody, running without hardware.\n");
    }
    //打开vga设备
    if ((vga_fd = open("/dev/vga_ball", O_RDWR)) == -1) {
        fprintf(stderr, "Warning: Could not open /dev/vga_ball.\n");
    }
    //关闭vga设备
    if (vga_fd != -1) close(vga_fd);
    // 4. 为历史缓冲区分配内存 (只分配坐标所需的内存)
    for (int i = 0; i < MAX_HISTORY; i++) {
        history[i] = malloc(sizeof(body_pos_t) * MAX_BODIES);
        if (!history[i]) {
            fprintf(stderr, "Memory allocation failed for history buffer.\n");
            exit(1);
        }
    }

    // Use the current system time as the random seed
    srand(time(NULL));

    fbclear(); // Clear the screen
    //fbinit_bodyshape(); // 初始化圆形掩码

    // 初始化第一次状态
    reset_system();
    draw_ui();

    // 启动键盘监听线程
    pthread_t kb_thread;
    if (keyboard != NULL) {
        pthread_create(&kb_thread, NULL, keyboard_handler, NULL);
    }

    int last_h_count = 0;

    // 5. 主事件循环
    while (running) {
        // 如果没有暂停，且用户停留在"最新帧"，则向硬件请求计算下一帧
        if (!is_paused && view_idx == h_count - 1) {
            if (nbody_fd != -1) {
                // A. 更新并写入动态配置参数 (gap)
                nbody_config_t config = {num_bodies, current_gap};
                ioctl(nbody_fd, NBODY_WRITE_CONFIG, &config);

                // B. 发送 Start 信号启动计算
                ioctl(nbody_fd, NBODY_START_CALC);
                
                // C. 轮询检查硬件计算是否完成
                int is_done = 0;
                do { 
                    ioctl(nbody_fd, NBODY_CHECK_DONE, &is_done); 
                } while (!is_done && running);

                // D. 读取计算结果，直接存入历史循环缓冲区
                body_pos_t *next_frame = history[h_head % MAX_HISTORY];
                ioctl(nbody_fd, NBODY_READ_POSITIONS, next_frame);
                
                // 更新环形缓冲区的游标
                h_head = (h_head + 1) % MAX_HISTORY;
                if (h_count < MAX_HISTORY) h_count++;
                view_idx = h_count - 1; // 视角始终跟踪最新帧

            } else {
                // 仅用于测试：如果没有硬件驱动时，避免死循环导致卡死
                usleep(30000); 
            }
        } 

        // 仅在暂停时主动刷新UI上的帧数指示，防止占用性能
        if (is_paused) {
            usleep(30000); 
            continue; 
        }
        
        // --- 渲染逻辑 ---
        if (last_h_count != h_count) {
            draw_body();
            last_h_count = h_count;
        }
    }

    // 6. 清理退出
    fbclear(); // Clear the screen

    if (keyboard != NULL) {
        pthread_cancel(kb_thread);
        pthread_join(kb_thread, NULL);
    }
    for(int i = 0; i < MAX_HISTORY; i++) {
        free(history[i]);
    }
    if(nbody_fd != -1) close(nbody_fd);
    
    return 0;
}

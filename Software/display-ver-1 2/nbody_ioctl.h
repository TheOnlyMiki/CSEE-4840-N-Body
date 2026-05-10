#ifndef _NBODY_IOCTL_H
#define _NBODY_IOCTL_H

#include <linux/ioctl.h>
#include <stdint.h>

/* 坐标 */
typedef struct {
    uint32_t x;
    uint32_t y;
} body_pos_t;

/* 仅用于启动时，向硬件发送的完整初始状态 */
typedef struct {
    uint32_t x;
    uint32_t y;
    int32_t vx;
    int32_t vy;
    uint32_t mass;
} body_init_t;

/* 传递给 Avalon 寄存器的动态配置参数 */
typedef struct {
    uint32_t num_bodies;
    uint32_t gap;
} nbody_config_t;

#define NBODY_MAGIC 'n'

/* ioctls 指令 */
#define NBODY_WRITE_INIT_DATA _IOW(NBODY_MAGIC, 1, body_init_t *) // 下发初始数组
#define NBODY_WRITE_CONFIG    _IOW(NBODY_MAGIC, 2, nbody_config_t)// 设置 body 数量和 gap
#define NBODY_READ_POSITIONS  _IOR(NBODY_MAGIC, 3, body_pos_t *)  // 读取下一帧坐标
#define NBODY_START_CALC      _IO(NBODY_MAGIC, 4)                 // 启动硬件计算
#define NBODY_CHECK_DONE      _IOR(NBODY_MAGIC, 5, int)           // 检查硬件是否完成
#define VGA_MAGIC 'v'
#define VGA_DRAW_FRAME      _IOW(VGA_MAGIC, 1, body_pos_t *)
#define VGA_SET_NUM_BODIES  _IOW(VGA_MAGIC, 2, uint32_t)
#define VGA_SET_RADIUS      _IOW(VGA_MAGIC, 3, uint8_t *)
#endif

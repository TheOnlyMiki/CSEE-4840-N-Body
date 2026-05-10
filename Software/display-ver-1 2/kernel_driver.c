#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "nbody_ioctl.h"

#define DEVICE_NAME "vga_ball"
#define CLASS_NAME  "vga"

/* 屏幕参数 */
#define DISPLAY_WIDTH    1024
#define DISPLAY_HEIGHT   736    /* 不包括底部UI的两行文字 */
#define WORDS_PER_ROW    (DISPLAY_WIDTH / 32)  /* 每行32个uint32 */

/* FPGA display RAM的物理地址，在Avalon bus上 */
#define FPGA_DISPLAY_BASE 0xFF200000
#define FPGA_DISPLAY_SIZE (DISPLAY_HEIGHT * WORDS_PER_ROW * sizeof(uint32_t))

/* 圆形mask，和原来fbinit_bodyshape()一样的逻辑 */
#define NUM_BODY_SHAPE 3
static uint8_t body_masks[NUM_BODY_SHAPE][9][9]; /* 最大半径4，包围盒9x9 */

/* Virtual framebuffer，1bit/像素 */
static uint32_t vfb[DISPLAY_HEIGHT * WORDS_PER_ROW];

/* FPGA RAM映射后的虚拟地址 */
static void __iomem *fpga_ptr = NULL;

/* 设备号和设备类 */
static int major_number;
static struct class  *vga_class  = NULL;
static struct device *vga_device = NULL;

/* ============================================================
 * set_pixel: 把屏幕上(x,y)这个像素设为1或0
 * ============================================================ */
static void set_pixel(int x, int y, int val)
{
    int index, bit;

    /* 越界检查 */
    if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT)
        return;

    index = y * WORDS_PER_ROW + x / 32;  /* 在第几个uint32 */
    bit   = x % 32;                       /* 在这个uint32的第几位 */

    if (val)
        vfb[index] |=  (1u << bit);  /* 点亮 */
    else
        vfb[index] &= ~(1u << bit);  /* 熄灭 */
}

/* ============================================================
 * init_bodyshape: 预计算圆形mask
 * 和原来fbinit_bodyshape()完全一样的逻辑
 * ============================================================ */
static void init_bodyshape(void)
{
    int radii[NUM_BODY_SHAPE] = {2, 3, 4};
    int i, x, y, dx, dy, r, center;

    memset(body_masks, 0, sizeof(body_masks));

    for (i = 0; i < NUM_BODY_SHAPE; i++) {
        r      = radii[i];
        center = r;
        for (y = 0; y <= 2 * r; y++) {
            for (x = 0; x <= 2 * r; x++) {
                dx = x - center;
                dy = y - center;
                if (dx * dx + dy * dy <= r * r + r)
                    body_masks[i][y][x] = 1;
            }
        }
    }
}

/* ============================================================
 * draw_body_hw: 用set_pixel画一个球
 * 和原来fbdrawbody()完全一样的逻辑
 * 只是把fbdraw_pixel()换成了set_pixel()
 * ============================================================ */
static void draw_body_hw(int cx, int cy, int radius_idx)
{
    int radii[NUM_BODY_SHAPE] = {2, 3, 4};
    int radius, size, x, y, screen_x, screen_y;

    if (radius_idx < 0 || radius_idx >= NUM_BODY_SHAPE)
        radius_idx = 0;

    radius = radii[radius_idx];
    size   = 2 * radius + 1;

    /* 边界检查 */
    if (cx - radius < 0 || cx + radius >= DISPLAY_WIDTH ||
        cy - radius < 0 || cy + radius >= DISPLAY_HEIGHT)
        return;

    for (y = 0; y < size; y++) {
        screen_y = cy - radius + y;
        for (x = 0; x < size; x++) {
            if (body_masks[radius_idx][y][x]) {
                screen_x = cx - radius + x;
                set_pixel(screen_x, screen_y, 1);  /* 白色 */
            }
        }
    }
}

/* ============================================================
 * flush_to_fpga: 把virtual framebuffer推到FPGA RAM
 * ============================================================ */
static void flush_to_fpga(void)
{
    if (fpga_ptr)
        memcpy_toio(fpga_ptr, vfb, sizeof(vfb));
}

/* ============================================================
 * ioctl handler
 * ============================================================ */
static long vga_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    body_pos_t *positions;
    uint32_t    num;
    int         i, radius_idx;

    switch (cmd) {

    case VGA_DRAW_FRAME:
        /*
         * arg是userspace传来的 body_pos_t 数组指针
         * 但我们还需要知道有多少个球
         * 所以先用一个简单方法：
         * 把num_bodies也存在driver里
         * 通过另一个ioctl设置（见下面VGA_SET_NUM_BODIES）
         */
        num = vga_num_bodies;  /* driver内部记录的球数 */

        /* 分配临时buffer接收userspace数据 */
        positions = kmalloc(sizeof(body_pos_t) * num, GFP_KERNEL);
        if (!positions)
            return -ENOMEM;

        /* 从userspace拷贝坐标数组 */
        if (copy_from_user(positions, (void __user *)arg,
                           sizeof(body_pos_t) * num)) {
            kfree(positions);
            return -EFAULT;
        }

        /* 1. 清空virtual framebuffer */
        memset(vfb, 0, sizeof(vfb));

        /* 2. 画每个球 */
        for (i = 0; i < num; i++) {
            radius_idx = vga_radius_idx[i]; /* 每个球的半径，另一个ioctl设置 */
            draw_body_hw(positions[i].x, positions[i].y, radius_idx);
        }

        /* 3. 推到FPGA RAM */
        flush_to_fpga();

        kfree(positions);
        return 0;

    case VGA_SET_NUM_BODIES:
        /* 设置球的数量 */
        if (copy_from_user(&vga_num_bodies, (void __user *)arg,
                           sizeof(uint32_t)))
            return -EFAULT;
        return 0;

    case VGA_SET_RADIUS:
        /* 设置每个球的半径idx数组 */
        if (copy_from_user(vga_radius_idx, (void __user *)arg,
                           sizeof(uint8_t) * vga_num_bodies))
            return -EFAULT;
        return 0;

    default:
        return -EINVAL;
    }
}

/* ============================================================
 * file operations
 * ============================================================ */
static int     vga_open   (struct inode *i, struct file *f) { return 0; }
static int     vga_release(struct inode *i, struct file *f) { return 0; }

static struct file_operations vga_fops = {
    .owner          = THIS_MODULE,
    .open           = vga_open,
    .release        = vga_release,
    .unlocked_ioctl = vga_ioctl,
};

/* ============================================================
 * module init / exit
 * ============================================================ */
static uint32_t vga_num_bodies = 0;
static uint8_t  vga_radius_idx[1024];

static int __init vga_init(void)
{
    /* 1. 注册字符设备 */
    major_number = register_chrdev(0, DEVICE_NAME, &vga_fops);
    if (major_number < 0) {
        printk(KERN_ALERT "vga_ball: failed to register device\n");
        return major_number;
    }

    /* 2. 创建设备类 */
    vga_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(vga_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(vga_class);
    }

    /* 3. 创建设备文件 /dev/vga_ball */
    vga_device = device_create(vga_class, NULL,
                               MKDEV(major_number, 0),
                               NULL, DEVICE_NAME);
    if (IS_ERR(vga_device)) {
        class_destroy(vga_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(vga_device);
    }

    /* 4. 映射FPGA RAM地址 */
    fpga_ptr = ioremap(FPGA_DISPLAY_BASE, FPGA_DISPLAY_SIZE);
    if (!fpga_ptr) {
        printk(KERN_ALERT "vga_ball: ioremap failed\n");
        device_destroy(vga_class, MKDEV(major_number, 0));
        class_destroy(vga_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return -ENOMEM;
    }

    /* 5. 初始化圆形mask */
    init_bodyshape();

    /* 6. 清空virtual framebuffer */
    memset(vfb, 0, sizeof(vfb));

    printk(KERN_INFO "vga_ball: driver loaded\n");
    return 0;
}

static void __exit vga_exit(void)
{
    iounmap(fpga_ptr);
    device_destroy(vga_class, MKDEV(major_number, 0));
    class_destroy(vga_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "vga_ball: driver unloaded\n");
}

module_init(vga_init);
module_exit(vga_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("csee4840");
MODULE_DESCRIPTION("VGA N-body display driver");
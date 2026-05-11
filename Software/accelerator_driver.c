/*
 * nbody.c - Linux 内核驱动，用于 N-Body FPGA 加速器
 *
 * 仿照 lab3 的 vga_ball.c 编写
 * 通过 Avalon 内存映射接口与 FPGA 通信
 *
 * 寄存器映射 (design doc Table 1):
 *   0x00  GO        W   启动计算（脉冲高电平）
 *   0x01  N_BODIES  W   星体数量
 *   0x02  GAP       W   时间步间隔
 *   0x03  X_IN      W   输入 X 位置
 *   0x04  Y_IN      W   输入 Y 位置
 *   0x05  M_IN      W   输入质量
 *   0x06  VX_IN     W   输入 X 速度
 *   0x07  VY_IN     W   输入 Y 速度（写此寄存器触发星体提交）
 *   0x08  DONE      R   计算完成标志
 *   0x09  READ      W   软件确认读取
 *   0x0A  OUT_X     R   输出 X 位置
 *   0x0B  OUT_Y     R   输出 Y 位置（读此寄存器触发输出指针递增）
 *
 * 使用方法:
 *   make && insmod nbody.ko
 *   （用户态通过 /dev/nbody 进行 ioctl 调用）
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "nbody_ioctl.h"

#define DRIVER_NAME "nbody"

/* ---------- 寄存器字节偏移（每个寄存器 4 字节，32-bit word） ---------- */
#define REG_GO       (0x00 * 4)   /* 0x00 */
#define REG_N_BODIES (0x01 * 4)   /* 0x04 */
#define REG_GAP      (0x02 * 4)   /* 0x08 */
#define REG_X_IN     (0x03 * 4)   /* 0x0C */
#define REG_Y_IN     (0x04 * 4)   /* 0x10 */
#define REG_M_IN     (0x05 * 4)   /* 0x14 */
#define REG_VX_IN    (0x06 * 4)   /* 0x18 */
#define REG_VY_IN    (0x07 * 4)   /* 0x1C */  /* 写此寄存器提交一个星体 */
#define REG_DONE     (0x08 * 4)   /* 0x20 */
#define REG_READ     (0x09 * 4)   /* 0x24 */
#define REG_OUT_X    (0x0A * 4)   /* 0x28 */
#define REG_OUT_Y    (0x0B * 4)   /* 0x2C */  /* 读此寄存器输出指针自动+1 */

/* ---------- 驱动内部设备状态 ---------- */
struct nbody_dev {
    struct resource  res;       /* 从设备树获取的物理地址资源 */
    void __iomem    *virtbase;  /* ioremap 后的虚拟基地址 */
    unsigned int     num_bodies;
    unsigned int     gap;
} dev;

/* ---------- 寄存器读写辅助宏 ---------- */
#define REG_WRITE(offset, val) iowrite32((val), dev.virtbase + (offset))
#define REG_READ(offset)       ioread32(dev.virtbase + (offset))

/* ================================================================
 * 底层寄存器操作函数（对应 design doc 5.1 节描述的协议）
 * ================================================================ */

/*
 * write_config - 写入 N_BODIES 和 GAP 寄存器
 */
static void write_config(nbody_config_t *cfg)
{
    dev.num_bodies = cfg->num_bodies;
    dev.gap        = cfg->gap;
    REG_WRITE(REG_N_BODIES, cfg->num_bodies);
    REG_WRITE(REG_GAP,      cfg->gap);
}

/*
 * write_bodies - 顺序将所有星体数据写入 FPGA
 *
 * 协议：每个星体按 X_IN → Y_IN → M_IN → VX_IN → VY_IN 顺序写入
 * 写入 VY_IN 时 FPGA 自动提交该星体并将输入指针 +1
 */
static int write_bodies(nbody_bodies_arg_t __user *uarg)
{
    nbody_bodies_arg_t arg;
    nbody_particle_t  *kbuf;
    unsigned int       i;
    int                ret = 0;

    /* 从用户空间复制结构体（含指针和数量） */
    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EACCES;

    if (arg.count == 0 || arg.count > 4096)
        return -EINVAL;

    /* 分配内核缓冲区，再从用户空间复制星体数组 */
    kbuf = kmalloc(arg.count * sizeof(nbody_particle_t), GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    if (copy_from_user(kbuf, arg.particles,
                       arg.count * sizeof(nbody_particle_t))) {
        ret = -EACCES;
        goto out;
    }

    /* 按协议顺序写入每个星体 */
    for (i = 0; i < arg.count; i++) {
        REG_WRITE(REG_X_IN,  kbuf[i].x);
        REG_WRITE(REG_Y_IN,  kbuf[i].y);
        REG_WRITE(REG_M_IN,  kbuf[i].mass);
        REG_WRITE(REG_VX_IN, kbuf[i].vx);
        REG_WRITE(REG_VY_IN, kbuf[i].vy); /* 最后写 VY_IN，触发提交 */
    }

out:
    kfree(kbuf);
    return ret;
}

/*
 * start_calc - 向 GO 寄存器写 1，启动计算
 * 同时隐式重置 FPGA 内部的输入/输出指针
 */
static void start_calc(void)
{
    REG_WRITE(REG_GO, 1);
}
/* GO=0 only called on program exit */
static void stop_calc(void)
{
    REG_WRITE(REG_GO, 0);
}
/*
 * check_done - 轮询 DONE 寄存器
 * 返回 1 表示计算完成，0 表示仍在计算
 */
static int check_done(void)
{
    return (REG_READ(REG_DONE) != 0) ? 1 : 0;
}

/*
 * read_results - 读取所有星体的新坐标
 *
 * 协议：依次读 OUT_X / OUT_Y，读 OUT_Y 时 FPGA 输出指针自动 +1
 */
static int read_results(nbody_read_arg_t __user *uarg)
{
    nbody_read_arg_t  arg;
    nbody_result_t   *kbuf;
    unsigned int      i;
    int               ret = 0;

    if (copy_from_user(&arg, uarg, sizeof(arg)))
        return -EACCES;

    if (arg.count == 0 || arg.count > 4096)
        return -EINVAL;

    kbuf = kmalloc(arg.count * sizeof(nbody_result_t), GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    /* 按协议顺序读出每个星体：先 OUT_X 再 OUT_Y */
    for (i = 0; i < arg.count; i++) {
        kbuf[i].x = REG_READ(REG_OUT_X);
        kbuf[i].y = REG_READ(REG_OUT_Y); /* 读 OUT_Y 触发输出指针 +1 */
    }

    /* 将结果复制回用户空间 */
    if (copy_to_user(arg.results, kbuf,
                     arg.count * sizeof(nbody_result_t)))
        ret = -EACCES;

    kfree(kbuf);
    return ret;
}

/* READ=1 stays high until clear_read() at start of next frame */
static void ack_read(void)
{
    REG_WRITE(REG_READ, 1);
}
/* READ=0 called at start of every frame */
static void clear_read(void)
{
    REG_WRITE(REG_READ, 0);
}
/* returns 1 when DONE=0, meaning FPGA has locked output data */
static int poll_done_low(void)
{
    return (REG_READ(REG_DONE) == 0) ? 1 : 0;
}
/* ================================================================
 * ioctl 分发函数
 * ================================================================ */
static long nbody_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    int done_val;

    switch (cmd) {

    /* 写入 num_bodies 和 gap */
    case NBODY_WRITE_CONFIG:
    {
        nbody_config_t cfg;
        if (copy_from_user(&cfg, (nbody_config_t __user *)arg, sizeof(cfg)))
            return -EACCES;
        write_config(&cfg);
        break;
    }

    /* 顺序写入所有星体初始数据 */
    case NBODY_WRITE_BODIES:
        return write_bodies((nbody_bodies_arg_t __user *)arg);

    /* 启动计算（脉冲 GO） */
    case NBODY_START_CALC:
        start_calc();
        break;

    /* 轮询 DONE，将结果（0 或 1）复制回用户空间 */
    case NBODY_CHECK_DONE:
        done_val = check_done();
        if (copy_to_user((int __user *)arg, &done_val, sizeof(done_val)))
            return -EACCES;
        break;

    /* 读取所有星体的新坐标 */
    case NBODY_READ_RESULTS:
        return read_results((nbody_read_arg_t __user *)arg);

    /* 确认读取完毕（READ 寄存器脉冲） */
    case NBODY_ACK_READ:
        ack_read();
        break;


    case NBODY_STOP_CALC:
        stop_calc();
        break;

    case NBODY_CLEAR_READ:
        clear_read();
        break;

    case NBODY_POLL_DONE_LOW:
        done_val = poll_done_low();
        if (copy_to_user((int __user *)arg, &done_val, sizeof(done_val)))
            return -EACCES;
    break;

/* NBODY_SOFT_RESET: GO=0 then GO=1 to reset FPGA state */
    case NBODY_SOFT_RESET:
        stop_calc();
        REG_WRITE(REG_READ, 0);
        start_calc();
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/* ================================================================
 * file_operations
 * ================================================================ */
static const struct file_operations nbody_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = nbody_ioctl,
};

static struct miscdevice nbody_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DRIVER_NAME,
    .fops  = &nbody_fops,
};

/* ================================================================
 * 平台驱动 probe / remove
 * ================================================================ */
static int __init nbody_probe(struct platform_device *pdev)
{
    int ret;

    /* 注册 misc 设备（创建 /dev/nbody） */
    ret = misc_register(&nbody_misc_device);
    if (ret) {
        pr_err(DRIVER_NAME ": misc_register failed (%d)\n", ret);
        return ret;
    }

    /* 从设备树获取物理地址资源 */
    ret = of_address_to_resource(pdev->dev.of_node, 0, &dev.res);
    if (ret) {
        pr_err(DRIVER_NAME ": of_address_to_resource failed\n");
        ret = -ENOENT;
        goto out_deregister;
    }

    /* 申请内存区域 */
    if (request_mem_region(dev.res.start,
                           resource_size(&dev.res),
                           DRIVER_NAME) == NULL) {
        pr_err(DRIVER_NAME ": request_mem_region failed\n");
        ret = -EBUSY;
        goto out_deregister;
    }

    /* ioremap 映射到虚拟地址 */
    dev.virtbase = of_iomap(pdev->dev.of_node, 0);
    if (dev.virtbase == NULL) {
        pr_err(DRIVER_NAME ": of_iomap failed\n");
        ret = -ENOMEM;
        goto out_release_mem_region;
    }

    pr_info(DRIVER_NAME ": mapped FPGA registers at phys=0x%08llx virt=%p\n",
            (unsigned long long)dev.res.start, dev.virtbase);

    /* 初始化：让 FPGA 处于已知状态 */
    REG_WRITE(REG_GO,      0);
    REG_WRITE(REG_N_BODIES, 0);
    REG_WRITE(REG_GAP,     1);
    REG_WRITE(REG_READ,    0);

    return 0;

out_release_mem_region:
    release_mem_region(dev.res.start, resource_size(&dev.res));
out_deregister:
    misc_deregister(&nbody_misc_device);
    return ret;
}

static int nbody_remove(struct platform_device *pdev)
{
    iounmap(dev.virtbase);
    release_mem_region(dev.res.start, resource_size(&dev.res));
    misc_deregister(&nbody_misc_device);
    return 0;
}

/* ================================================================
 * 设备树匹配 & 模块注册
 * ================================================================ */
#ifdef CONFIG_OF
static const struct of_device_id nbody_of_match[] = {
    { .compatible = "csee4840,nbody-1.0" },
    {},
};
MODULE_DEVICE_TABLE(of, nbody_of_match);
#endif

static struct platform_driver nbody_driver = {
    .driver = {
        .name           = DRIVER_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(nbody_of_match),
    },
    .remove = __exit_p(nbody_remove),
};

static int __init nbody_init(void)
{
    pr_info(DRIVER_NAME ": init\n");
    return platform_driver_probe(&nbody_driver, nbody_probe);
}

static void __exit nbody_exit(void)
{
    platform_driver_unregister(&nbody_driver);
    pr_info(DRIVER_NAME ": exit\n");
}

module_init(nbody_init);
module_exit(nbody_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSEE4840 Group");
MODULE_DESCRIPTION("N-Body FPGA Accelerator Driver");

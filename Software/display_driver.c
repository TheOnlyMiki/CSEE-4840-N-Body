/*
 * Packed bitmap display Avalon-MM Linux driver.
 *
 * The kernel only transfers the userspace-rendered 640x480 1-bpp framebuffer
 * to vga_bitmap_avmm.sv. It does not render particles or text.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "display_ioctl.h"

#define DRIVER_NAME "nbody_display"

struct display_dev {
    struct resource res;
    void __iomem *virtbase;
    uint32_t *kbuf;
};

static struct display_dev dev;

static long display_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    uint32_t i;

    switch (cmd) {
    case DISPLAY_WRITE_FRAME:
        if (copy_from_user(dev.kbuf, (uint32_t __user *)arg,
                           DISPLAY_WORDS * sizeof(uint32_t)))
            return -EFAULT;

        /*
         * vga_bitmap_avmm.sv uses word addresses 0..9599, so byte offset
         * is i * 4. Framebuffer packing is word = y * 20 + x / 32,
         * bit = x % 32, LSB-first.
         */
        for (i = 0; i < DISPLAY_WORDS; i++)
            iowrite32(dev.kbuf[i], dev.virtbase + i * 4);
        return 0;

    case DISPLAY_CLEAR:
        for (i = 0; i < DISPLAY_WORDS; i++)
            iowrite32(0, dev.virtbase + i * 4);
        return 0;

    default:
        return -EINVAL;
    }
}

static const struct file_operations display_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = display_ioctl,
};

static struct miscdevice display_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "nbody_display",
    .fops = &display_fops,
};

static int display_probe(struct platform_device *pdev)
{
    int ret;

    ret = of_address_to_resource(pdev->dev.of_node, 0, &dev.res);
    if (ret)
        return ret;

    if (!request_mem_region(dev.res.start, resource_size(&dev.res), DRIVER_NAME))
        return -EBUSY;

    dev.virtbase = of_iomap(pdev->dev.of_node, 0);
    if (!dev.virtbase) {
        ret = -ENOMEM;
        goto out_release;
    }

    dev.kbuf = kmalloc(DISPLAY_WORDS * sizeof(uint32_t), GFP_KERNEL);
    if (!dev.kbuf) {
        ret = -ENOMEM;
        goto out_unmap;
    }

    ret = misc_register(&display_misc_device);
    if (ret)
        goto out_free;

    pr_info("display driver registered /dev/nbody_display\n");
    return 0;

out_free:
    kfree(dev.kbuf);
out_unmap:
    iounmap(dev.virtbase);
out_release:
    release_mem_region(dev.res.start, resource_size(&dev.res));
    return ret;
}

static int display_remove(struct platform_device *pdev)
{
    misc_deregister(&display_misc_device);
    kfree(dev.kbuf);
    if (dev.virtbase)
        iounmap(dev.virtbase);
    release_mem_region(dev.res.start, resource_size(&dev.res));
    return 0;
}

static const struct of_device_id display_of_match[] = {
    { .compatible = "csee4840,vga_bitmap_avmm-1.0" },
    {},
};
MODULE_DEVICE_TABLE(of, display_of_match);

static struct platform_driver display_driver = {
    .probe = display_probe,
    .remove = display_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = display_of_match,
    },
};

module_platform_driver(display_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSEE4840 Group");
MODULE_DESCRIPTION("Packed VGA bitmap Avalon-MM display driver");

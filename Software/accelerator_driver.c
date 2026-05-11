/*
 * N-body Avalon-MM Linux driver.
 *
 * This follows nbody_accel_avmm.sv exactly. Register addresses below are
 * byte offsets for ioread32/iowrite32; the hardware Avalon address is a
 * 32-bit word address, so every register is word_address * 4.
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
#include "nbody_ioctl.h"

#define DRIVER_NAME "nbody"

#define REG_GO        (0x00 * 4)
#define REG_N_BODIES  (0x01 * 4)
#define REG_GAP       (0x02 * 4)
#define REG_X_IN      (0x03 * 4)
#define REG_Y_IN      (0x04 * 4)
#define REG_M_IN      (0x05 * 4)
#define REG_VX_IN     (0x06 * 4)
#define REG_VY_IN     (0x07 * 4)
#define REG_DONE      (0x08 * 4)
#define REG_READ      (0x09 * 4)
#define REG_OUT_X     (0x0A * 4)
#define REG_OUT_Y     (0x0B * 4)

struct nbody_dev {
    struct resource res;
    void __iomem *virtbase;
    uint32_t num_bodies;
    uint32_t gap;
};

static struct nbody_dev dev;

static int nbody_write_bodies(unsigned long arg)
{
    nbody_bodies_arg_t barg;
    nbody_particle_t *particles;
    size_t bytes;
    uint32_t i;

    if (copy_from_user(&barg, (void __user *)arg, sizeof(barg)))
        return -EFAULT;

    if (barg.count == 0 || barg.count > NBODY_MAX_BODIES || !barg.particles)
        return -EINVAL;

    bytes = barg.count * sizeof(*particles);
    particles = memdup_user((const void __user *)barg.particles, bytes);
    if (IS_ERR(particles))
        return PTR_ERR(particles);

    /*
     * N-body payloads are raw low-27-bit values. Writing VY_IN commits
     * the current body in nbody_accel_avmm.sv and advances input_ptr.
     */
    for (i = 0; i < barg.count; i++) {
        iowrite32(particles[i].x & NBODY_DATA_MASK, dev.virtbase + REG_X_IN);
        iowrite32(particles[i].y & NBODY_DATA_MASK, dev.virtbase + REG_Y_IN);
        iowrite32(particles[i].mass & NBODY_DATA_MASK, dev.virtbase + REG_M_IN);
        iowrite32(particles[i].vx & NBODY_DATA_MASK, dev.virtbase + REG_VX_IN);
        iowrite32(particles[i].vy & NBODY_DATA_MASK, dev.virtbase + REG_VY_IN);
    }

    kfree(particles);
    return 0;
}

static int nbody_read_results(unsigned long arg)
{
    nbody_read_arg_t rarg;
    nbody_result_t *results;
    size_t bytes;
    uint32_t i;
    int ret = 0;

    if (copy_from_user(&rarg, (void __user *)arg, sizeof(rarg)))
        return -EFAULT;

    if (rarg.count == 0 || rarg.count > NBODY_MAX_BODIES ||
        rarg.count > dev.num_bodies || !rarg.results)
        return -EINVAL;

    bytes = rarg.count * sizeof(*results);
    results = kmalloc(bytes, GFP_KERNEL);
    if (!results)
        return -ENOMEM;

    /*
     * Read OUT_X first, then OUT_Y. Only the OUT_Y read increments the
     * hardware output pointer.
     */
    for (i = 0; i < rarg.count; i++) {
        results[i].x = ioread32(dev.virtbase + REG_OUT_X) & NBODY_DATA_MASK;
        results[i].y = ioread32(dev.virtbase + REG_OUT_Y) & NBODY_DATA_MASK;
    }

    if (copy_to_user((void __user *)rarg.results, results, bytes))
        ret = -EFAULT;

    kfree(results);
    return ret;
}

static long nbody_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    nbody_config_t cfg;
    int done;

    switch (cmd) {
    case NBODY_WRITE_CONFIG:
        if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg)))
            return -EFAULT;
        if (cfg.num_bodies == 0 || cfg.num_bodies > NBODY_MAX_BODIES ||
            cfg.gap == 0)
            return -EINVAL;

        dev.num_bodies = cfg.num_bodies;
        dev.gap = cfg.gap;
        iowrite32(cfg.num_bodies, dev.virtbase + REG_N_BODIES);
        iowrite32(cfg.gap, dev.virtbase + REG_GAP);
        return 0;

    case NBODY_WRITE_BODIES:
        return nbody_write_bodies(arg);

    case NBODY_START_RUN:
        iowrite32(1, dev.virtbase + REG_GO);
        return 0;

    case NBODY_CHECK_DONE:
        done = ioread32(dev.virtbase + REG_DONE) & 1;
        if (copy_to_user((int __user *)arg, &done, sizeof(done)))
            return -EFAULT;
        return 0;

    case NBODY_READ_RESULTS:
        return nbody_read_results(arg);

    case NBODY_CLEAR_READ:
        iowrite32(0, dev.virtbase + REG_READ);
        return 0;

    case NBODY_STOP:
        /* GO is pulse-based in SV; best-effort stop is clearing READ. */
        iowrite32(0, dev.virtbase + REG_READ);
        return 0;

    case NBODY_SOFT_RESET:
        /*
         * There is no full software reset register. Clear READ/output_ptr;
         * userspace must reload config and bodies for a fresh simulation.
         */
        iowrite32(0, dev.virtbase + REG_READ);
        return 0;

    default:
        return -EINVAL;
    }
}

static const struct file_operations nbody_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = nbody_ioctl,
};

static struct miscdevice nbody_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "nbody",
    .fops = &nbody_fops,
};

static int nbody_probe(struct platform_device *pdev)
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

    ret = misc_register(&nbody_misc_device);
    if (ret)
        goto out_unmap;

    iowrite32(0, dev.virtbase + REG_READ);
    iowrite32(1, dev.virtbase + REG_GAP);
    pr_info("nbody driver registered /dev/nbody\n");
    return 0;

out_unmap:
    iounmap(dev.virtbase);
out_release:
    release_mem_region(dev.res.start, resource_size(&dev.res));
    return ret;
}

static int nbody_remove(struct platform_device *pdev)
{
    misc_deregister(&nbody_misc_device);
    if (dev.virtbase)
        iounmap(dev.virtbase);
    release_mem_region(dev.res.start, resource_size(&dev.res));
    return 0;
}

static const struct of_device_id nbody_of_match[] = {
    { .compatible = "csee4840,nbody_accel_avmm-1.0" },
    {},
};
MODULE_DEVICE_TABLE(of, nbody_of_match);

static struct platform_driver nbody_driver = {
    .probe = nbody_probe,
    .remove = nbody_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = nbody_of_match,
    },
};

module_platform_driver(nbody_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSEE4840 Group");
MODULE_DESCRIPTION("N-body Avalon-MM accelerator driver");

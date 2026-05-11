#ifndef _NBODY_IOCTL_H
#define _NBODY_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

#define NBODY_MAX_BODIES 1024
#define NBODY_DATA_W 27
#define NBODY_DATA_MASK 0x07ffffffu
#define NBODY_MAGIC 'n'

typedef struct {
    uint32_t x;      /* low 27 bits used */
    uint32_t y;      /* low 27 bits used */
    uint32_t mass;   /* low 27 bits used */
    uint32_t vx;     /* low 27 bits used */
    uint32_t vy;     /* low 27 bits used */
} nbody_particle_t;

typedef struct {
    uint32_t x;      /* low 27 bits from OUT_X */
    uint32_t y;      /* low 27 bits from OUT_Y */
} nbody_result_t;

typedef nbody_result_t body_pos_t;

typedef struct {
    uint32_t num_bodies;
    uint32_t gap;
} nbody_config_t;

typedef struct {
    const nbody_particle_t *particles;
    uint32_t count;
} nbody_bodies_arg_t;

typedef struct {
    nbody_result_t *results;
    uint32_t count;
} nbody_read_arg_t;

#define NBODY_WRITE_CONFIG   _IOW(NBODY_MAGIC, 1, nbody_config_t)
#define NBODY_WRITE_BODIES   _IOW(NBODY_MAGIC, 2, nbody_bodies_arg_t)
#define NBODY_START_RUN      _IO(NBODY_MAGIC, 3)
#define NBODY_CHECK_DONE     _IOR(NBODY_MAGIC, 4, int)
#define NBODY_READ_RESULTS   _IOWR(NBODY_MAGIC, 5, nbody_read_arg_t)
#define NBODY_CLEAR_READ     _IO(NBODY_MAGIC, 6)
#define NBODY_STOP           _IO(NBODY_MAGIC, 7)
#define NBODY_SOFT_RESET     _IO(NBODY_MAGIC, 8)

#define NBODY_WRITE_INIT_DATA NBODY_WRITE_BODIES
#define NBODY_READ_POSITIONS  NBODY_READ_RESULTS

#endif

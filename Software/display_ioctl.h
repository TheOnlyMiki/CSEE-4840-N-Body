#ifndef _DISPLAY_IOCTL_H
#define _DISPLAY_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

#define DISPLAY_WIDTH 640
#define DISPLAY_HEIGHT 480
#define DISPLAY_WORDS_PER_ROW 20
#define DISPLAY_WORDS 9600

#define DISPLAY_MAGIC 'd'

#define DISPLAY_WRITE_FRAME _IOW(DISPLAY_MAGIC, 1, uint32_t *)
#define DISPLAY_CLEAR       _IO(DISPLAY_MAGIC, 2)

#endif

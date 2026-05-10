#ifndef _FRAMEBUFFER_H
#define _FRAMEBUFFER_H

#define FBOPEN_DEV -1          /* Couldn't open the device */
#define FBOPEN_FSCREENINFO -2  /* Couldn't read the fixed info */
#define FBOPEN_VSCREENINFO -3  /* Couldn't read the variable info */
#define FBOPEN_MMAP -4         /* Couldn't mmap the framebuffer memory */
#define FBOPEN_BPP -5          /* Unexpected bits-per-pixel */

extern int fbopen(void);
extern void fbputchar(char, int, int);
extern void fbputs(const char *, int, int);

extern void fbdraw_pixel(unsigned char* pixel, int r, int g, int b);

extern void fbclear(void);

/* 初始化预计算的圆形位图掩码 */
/*extern void fbinit_bodyshape(void);

/* 使用预计算好的掩码画出星体，radius_idx 取值 0~4 (对应半径 1~5) */
/*extern void fbdrawbody(int cx, int cy, int radius_idx, int r, int g, int b);

/* Clear the screen's drawing area */
/*extern void fbclear_bodyscreen(void);

/* 清空底部 UI 区域 (Y坐标 448 ~ 479) */
extern void fbclear_ui(void);
// VGA kernel driver的文件描述符
extern int vga_fd;
#endif

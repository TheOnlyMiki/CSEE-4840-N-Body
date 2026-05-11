#ifndef _NBODY_H
#define _NBODY_H

#include <pthread.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include "nbody_ioctl.h"

#define MAX_BODIES NBODY_MAX_BODIES
#define MAX_HISTORY 32768
#define BODY_DISPLAY_W 640
#define BODY_DISPLAY_H 448
#define NBODY_POS_MIN (-200.0f)
#define NBODY_POS_MAX 200.0f
#define NBODY_MASS_MIN 0.002f
#define NBODY_MASS_MAX 0.01f
#define NBODY_GAP_MAX 50

extern int num_bodies;
extern int current_gap;
extern int is_paused;
extern int running;

extern uint32_t static_masses[MAX_BODIES];
extern body_pos_t *history[MAX_HISTORY];
extern int h_head;
extern int h_count;
extern int view_idx;

extern int nbody_fd;
extern struct libusb_device_handle *keyboard;
extern uint8_t endpoint_address;

extern pthread_mutex_t state_mutex;
extern pthread_cond_t frame_ready_cond;

int allocate_history(void);
void free_history(void);
int get_radius_idx(float mass);
int world_to_screen_x(float x);
int world_to_screen_y(float y);
void reset_system(void);
void *nbody_thread(void *arg);
void *keyboard_handler(void *arg);

#endif

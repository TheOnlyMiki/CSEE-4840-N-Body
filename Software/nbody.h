#ifndef _NBODY_H
#define _NBODY_H

#include <pthread.h>
#include <stdint.h>
#include "nbody_ioctl.h"

#define MAX_BODIES NBODY_MAX_BODIES
#define MAX_HISTORY 32768
#define BODY_DISPLAY_W 640
#define BODY_DISPLAY_H 448
#define NBODY_POS_MIN (-10.0f)
#define NBODY_POS_MAX 10.0f
#define NBODY_MASS_MIN 0.002f
#define NBODY_MASS_MAX 0.01f
#define NBODY_GAP_MAX 10

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

extern pthread_mutex_t state_mutex;
extern pthread_cond_t frame_ready_cond;

int allocate_history(void);
void free_history(void);
int get_radius_idx(float mass);
void nbody_set_gap_delta(int delta);
void nbody_show_frame_delta(int delta);
void nbody_toggle_pause(void);
void nbody_request_reset(void);
void nbody_request_quit(void);
void reset_system(void);
void *nbody_thread(void *arg);

#endif

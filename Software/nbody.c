#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "nbody.h"
#include "usbkeyboard.h"

int num_bodies = 0;
int current_gap = 1;
int is_paused = 0;
int running = 1;

uint32_t static_masses[MAX_BODIES];
body_pos_t *history[MAX_HISTORY];
int h_head = 0;
int h_count = 0;
int view_idx = 0;

int nbody_fd = -1;
struct libusb_device_handle *keyboard = NULL;
uint8_t endpoint_address;

pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t frame_ready_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t hw_mutex = PTHREAD_MUTEX_INITIALIZER;

int allocate_history(void)
{
    int i;

    for (i = 0; i < MAX_HISTORY; i++) {
        history[i] = malloc(MAX_BODIES * sizeof(body_pos_t));
        if (!history[i]) {
            free_history();
            return -1;
        }
    }
    return 0;
}

void free_history(void)
{
    int i;

    for (i = 0; i < MAX_HISTORY; i++) {
        free(history[i]);
        history[i] = NULL;
    }
}

int get_radius_idx(uint32_t mass)
{
    mass &= NBODY_DATA_MASK;
    if (mass < 16)
        return 0;
    if (mass < 32)
        return 1;
    if (mass < 48)
        return 2;
    return 3;
}

int raw27_to_screen_x(uint32_t raw)
{
    return (int)((raw & NBODY_DATA_MASK) % BODY_DISPLAY_W);
}

int raw27_to_screen_y(uint32_t raw)
{
    return (int)((raw & NBODY_DATA_MASK) % BODY_DISPLAY_H);
}

static uint32_t raw_velocity(void)
{
    int mag = (rand() % 5) + 1;

    if (rand() & 1)
        return (uint32_t)mag & NBODY_DATA_MASK;

    return ((uint32_t)(-mag)) & NBODY_DATA_MASK;
}

void reset_system(void)
{
    nbody_particle_t *init_particles;
    int local_num;
    int local_gap;
    int i;

    pthread_mutex_lock(&state_mutex);
    local_num = num_bodies;
    local_gap = current_gap;
    pthread_mutex_unlock(&state_mutex);

    init_particles = malloc(local_num * sizeof(*init_particles));
    if (!init_particles) {
        fprintf(stderr, "failed to allocate initial particles\n");
        return;
    }

    pthread_mutex_lock(&state_mutex);
    for (i = 0; i < local_num; i++) {
        uint32_t mass = (uint32_t)((rand() % 64) + 1);

        init_particles[i].x = (uint32_t)(rand() % BODY_DISPLAY_W) & NBODY_DATA_MASK;
        init_particles[i].y = (uint32_t)(rand() % BODY_DISPLAY_H) & NBODY_DATA_MASK;
        init_particles[i].mass = mass & NBODY_DATA_MASK;
        init_particles[i].vx = raw_velocity();
        init_particles[i].vy = raw_velocity();

        static_masses[i] = (uint32_t)get_radius_idx(mass);
        history[0][i].x = init_particles[i].x;
        history[0][i].y = init_particles[i].y;
    }
    pthread_mutex_unlock(&state_mutex);

    if (nbody_fd >= 0) {
        nbody_config_t cfg = {
            .num_bodies = (uint32_t)local_num,
            .gap = (uint32_t)local_gap,
        };
        nbody_bodies_arg_t barg = {
            .particles = init_particles,
            .count = (uint32_t)local_num,
        };

        pthread_mutex_lock(&hw_mutex);
        ioctl(nbody_fd, NBODY_CLEAR_READ);
        if (ioctl(nbody_fd, NBODY_WRITE_CONFIG, &cfg) < 0)
            perror("NBODY_WRITE_CONFIG");
        if (ioctl(nbody_fd, NBODY_WRITE_BODIES, &barg) < 0)
            perror("NBODY_WRITE_BODIES");
        pthread_mutex_unlock(&hw_mutex);
    }

    pthread_mutex_lock(&state_mutex);
    h_head = 1;
    h_count = 1;
    view_idx = 0;
    pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);

    free(init_particles);
}

void *nbody_thread(void *arg)
{
    (void)arg;

    nbody_fd = open("/dev/nbody", O_RDWR);
    if (nbody_fd < 0)
        perror("open /dev/nbody");

    reset_system();

    while (running) {
        int paused;
        int local_num;
        int next_idx;
        int done = 0;
        nbody_read_arg_t rarg;

        pthread_mutex_lock(&state_mutex);
        paused = is_paused;
        local_num = num_bodies;
        pthread_mutex_unlock(&state_mutex);

        if (paused || nbody_fd < 0) {
            usleep(10000);
            continue;
        }

        pthread_mutex_lock(&state_mutex);
        next_idx = h_head & (MAX_HISTORY - 1);
        pthread_mutex_unlock(&state_mutex);

        pthread_mutex_lock(&hw_mutex);
        if (ioctl(nbody_fd, NBODY_START_RUN) < 0) {
            perror("NBODY_START_RUN");
            pthread_mutex_unlock(&hw_mutex);
            usleep(10000);
            continue;
        }

        while (running && !done) {
            if (ioctl(nbody_fd, NBODY_CHECK_DONE, &done) < 0) {
                perror("NBODY_CHECK_DONE");
                break;
            }
            if (!done)
                usleep(1000);
        }

        if (!running) {
            pthread_mutex_unlock(&hw_mutex);
            break;
        }

        rarg.results = history[next_idx];
        rarg.count = (uint32_t)local_num;
        if (ioctl(nbody_fd, NBODY_READ_RESULTS, &rarg) < 0)
            perror("NBODY_READ_RESULTS");

        if (ioctl(nbody_fd, NBODY_CLEAR_READ) < 0)
            perror("NBODY_CLEAR_READ");
        pthread_mutex_unlock(&hw_mutex);

        pthread_mutex_lock(&state_mutex);
        h_head = (h_head + 1) & (MAX_HISTORY - 1);
        if (h_count < MAX_HISTORY)
            h_count++;
        view_idx = h_count - 1;
        pthread_cond_broadcast(&frame_ready_cond);
        pthread_mutex_unlock(&state_mutex);
    }

    if (nbody_fd >= 0) {
        pthread_mutex_lock(&hw_mutex);
        ioctl(nbody_fd, NBODY_CLEAR_READ);
        ioctl(nbody_fd, NBODY_STOP);
        pthread_mutex_unlock(&hw_mutex);
        close(nbody_fd);
        nbody_fd = -1;
    }

    return NULL;
}

void *keyboard_handler(void *arg)
{
    struct usb_keyboard_packet packet;
    uint8_t last_packet[8] = { 0 };

    (void)arg;

    while (running) {
        int transferred = 0;
        int rc;
        uint8_t keycode;

        rc = libusb_interrupt_transfer(keyboard, endpoint_address,
                                       (unsigned char *)&packet, sizeof(packet),
                                       &transferred, 50);
        if (rc == LIBUSB_ERROR_TIMEOUT)
            continue;
        if (rc != 0 || transferred != sizeof(packet))
            continue;

        if (memcmp(&packet, last_packet, sizeof(packet)) == 0)
            continue;
        memcpy(last_packet, &packet, sizeof(packet));

        keycode = packet.keycode[0];
        if (keycode == 0)
            continue;

        if (keycode == 0x2c) {
            pthread_mutex_lock(&state_mutex);
            is_paused = !is_paused;
            pthread_cond_broadcast(&frame_ready_cond);
            pthread_mutex_unlock(&state_mutex);
        } else if (keycode == 0x1a || keycode == 0x16) {
            nbody_config_t cfg;

            pthread_mutex_lock(&state_mutex);
            if (keycode == 0x1a && current_gap < 255)
                current_gap++;
            else if (keycode == 0x16 && current_gap > 1)
                current_gap--;
            cfg.num_bodies = (uint32_t)num_bodies;
            cfg.gap = (uint32_t)current_gap;
            pthread_mutex_unlock(&state_mutex);

            if (nbody_fd >= 0) {
                pthread_mutex_lock(&hw_mutex);
                ioctl(nbody_fd, NBODY_WRITE_CONFIG, &cfg);
                pthread_mutex_unlock(&hw_mutex);
            }
        } else if (keycode == 0x04) {
            pthread_mutex_lock(&state_mutex);
            is_paused = 1;
            if (view_idx > 0)
                view_idx--;
            pthread_cond_broadcast(&frame_ready_cond);
            pthread_mutex_unlock(&state_mutex);
        } else if (keycode == 0x07) {
            pthread_mutex_lock(&state_mutex);
            is_paused = 1;
            if (view_idx < h_count - 1)
                view_idx++;
            pthread_cond_broadcast(&frame_ready_cond);
            pthread_mutex_unlock(&state_mutex);
        } else if (keycode == 0x15) {
            pthread_mutex_lock(&state_mutex);
            is_paused = 1;
            pthread_mutex_unlock(&state_mutex);
            reset_system();
        } else if (keycode == 0x14) {
            pthread_mutex_lock(&state_mutex);
            running = 0;
            pthread_cond_broadcast(&frame_ready_cond);
            pthread_mutex_unlock(&state_mutex);
        }
    }

    return NULL;
}

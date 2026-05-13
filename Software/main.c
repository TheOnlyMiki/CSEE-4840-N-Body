#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "display.h"
#include "keyboard.h"
#include "nbody.h"

int main(int argc, char **argv)
{
    pthread_t accelerator_thread;
    pthread_t vga_thread;
    pthread_t kb_thread;
    int have_keyboard = 0;

    // Check command-line arguments
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <Num Bodies> <Gap>\n", argv[0]);
        return 1;
    }

    // Setting num_bodies and current_gap
    num_bodies = atoi(argv[1]);
    current_gap = atoi(argv[2]);
    if (num_bodies < 5 || num_bodies > MAX_BODIES) {
        fprintf(stderr, "Num Bodies must be between 5 and %d\n", MAX_BODIES);
        return 1;
    }
    if (current_gap < 1 || current_gap > NBODY_GAP_MAX) {
        fprintf(stderr, "Gap must be between 1 and %d\n", NBODY_GAP_MAX);
        return 1;
    }

    // Allocate history buffer frames with body positions
    if (allocate_history() < 0) {
        fprintf(stderr, "Could not allocate history ring (%d frames)\n", MAX_HISTORY);
        return 1;
    }

    // Initialize random seed
    srand((unsigned int) time(NULL));

    // Create Nbody (Accelerator Simulation) Thread
    if (pthread_create(&accelerator_thread, NULL, nbody_thread, NULL) != 0) {
        perror("pthread_create nbody_thread");
        free_history();
        return 1;
    }

    // Create Display Thread
    if (pthread_create(&vga_thread, NULL, display_thread, NULL) != 0) {
        perror("pthread_create display_thread");
        pthread_mutex_lock(&state_mutex);
        running = 0;
        //pthread_cond_broadcast(&frame_ready_cond);
        pthread_mutex_unlock(&state_mutex);
        pthread_join(accelerator_thread, NULL);
        free_history();
        return 1;
    }

    // Open keyboard device and create Keyboard Thread
    if (keyboard_open())
        have_keyboard = 1;
    if (have_keyboard && pthread_create(&kb_thread, NULL, keyboard_handler, NULL) != 0) {
        perror("pthread_create keyboard_handler");
        have_keyboard = 0;
        free_history();
        return 1;
    }

    // Waiting Nbody thread to be destory
    pthread_join(accelerator_thread, NULL);

    pthread_mutex_lock(&state_mutex);
    running = 0;
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);

    // Waiting Keyboard and Display thread to be destory
    pthread_join(kb_thread, NULL);
    pthread_join(vga_thread, NULL);

    // Clear the history frame buffer
    free_history();

    return 0;
}

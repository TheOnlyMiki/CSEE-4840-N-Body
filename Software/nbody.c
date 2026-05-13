#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "nbody.h"

int num_bodies = 0;
int current_gap = 1;
int is_paused = 0;
int running = 1;

uint32_t static_masses[MAX_BODIES];
body_pos_t *history[MAX_HISTORY];
int h_head = 0;
int h_count = 0;
int view_idx = 0;

// Used to distinguish between old and new simulations following a reset
static unsigned int sim_generation = 0;
// Used to pause after a reset, awaiting user input to continue
static int reset_waiting_for_play = 0;

int nbody_fd = -1;

pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t frame_ready_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t hw_mutex = PTHREAD_MUTEX_INITIALIZER;

// Allocate memory for history frames with body positions
int allocate_history(void)
{
    for (int i = 0; i < MAX_HISTORY; i++) {
        history[i] = malloc(MAX_BODIES * sizeof(body_pos_t));
        if (!history[i]) {
            free_history();
            return -1;
        }
    }
    return 0;
}

// Release the memory for all history frames
void free_history(void)
{
    for (int i = 0; i < MAX_HISTORY; i++) {
        free(history[i]);
        history[i] = NULL;
    }
}

// Generate a random float within the range [min, max]
static float random_range(float min, float max)
{
    return min + (max - min) * ((float)rand() / (float)RAND_MAX);
}

// Map the mass to a radius index for display purposes
int get_radius_idx(float mass)
{
    float normalized = (mass - NBODY_MASS_MIN) / (NBODY_MASS_MAX - NBODY_MASS_MIN);

    if (normalized < 0.25f)
        return 0;
    if (normalized < 0.50f)
        return 1;
    if (normalized < 0.75f)
        return 2;
    return 3;
}

// Update current_gap in the software, and REG_GAP/REG_N_BODIES in the hardware
void nbody_set_gap_delta(int delta)
{
    nbody_config_t cfg;

    // Modify current_gap, then save the new configuration to cfg
    pthread_mutex_lock(&state_mutex);
    // If W is pressed, delta > 0, and current_gap is increased
    if (delta > 0 && current_gap < NBODY_GAP_MAX) {
        current_gap++;
    } 
    // If S is pressed, delta < 0, and current_gap is decreased 
    else if (delta < 0 && current_gap > 1) {
        current_gap--;
    }

    cfg.num_bodies = (uint32_t) num_bodies;
    cfg.gap = (uint32_t) current_gap;
    pthread_mutex_unlock(&state_mutex);

    // Write the new configuration to the hardware device /dev/nbody using ioctl
    if (nbody_fd >= 0) {
        pthread_mutex_lock(&hw_mutex);
        ioctl(nbody_fd, NBODY_WRITE_CONFIG, &cfg);
        pthread_mutex_unlock(&hw_mutex);
    }
}

// Used for browsing history frames
void nbody_show_frame_delta(int delta)
{
    pthread_mutex_lock(&state_mutex);
    // User manually advances the frame, the simulation automatically pauses
    is_paused = 1;

    // If A is pressed, delta < 0, and the previous frame is displayed
    if (delta < 0 && view_idx > 0) {
        view_idx--;
    } 
    // If D is pressed, delta > 0, and the next frame is displayed
    else if (delta > 0 && view_idx < h_count - 1) {
        view_idx++;
    }
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);
}

/*
Function for SPACE (Play/Pause) being pressed
Status: Play -> Paused / Paused -> Play
*/
void nbody_toggle_pause(void)
{
    pthread_mutex_lock(&state_mutex);
    is_paused = !is_paused;
    // Clear reset waiting status once user switches from the paused to the play 
    if (!is_paused)
        reset_waiting_for_play = 0;
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);
}

// Function for R (Reset) being pressed
void nbody_request_reset(void)
{
    pthread_mutex_lock(&state_mutex);
    // If a reset has already been performed and the system is waiting for 
    // the user to press Play, do not perform another reset
    if (reset_waiting_for_play) {
        pthread_mutex_unlock(&state_mutex);
        return;
    }
    // The simulation automatically pauses
    is_paused = 1;
    reset_waiting_for_play = 1;
    // Rrevent old results from prior to a reset from being written back to history
    sim_generation++;
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);

    // Regenerate initial particles and write to hardware
    reset_system();
}

// Function for Q (Exit) being pressed
void nbody_request_quit(void)
{
    pthread_mutex_lock(&state_mutex);
    // All threads monitor this variable in their main loops
    // Consequently, this causes the entire program to exit
    running = 0;
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);
}

// Responsible for initializing a new round of simulation
void reset_system(void)
{
    nbody_particle_t *init_particles;

    // Copy global variables to local variables
    pthread_mutex_lock(&state_mutex);
    int local_num = num_bodies;
    int local_gap = current_gap;
    pthread_mutex_unlock(&state_mutex);

    // Allocate the initial particle array
    init_particles = malloc(local_num * sizeof(*init_particles));
    if (!init_particles) {
        fprintf(stderr, "failed to allocate initial particles\n");
        return;
    }

    // Randomly Generate Initial State
    pthread_mutex_lock(&state_mutex);
    for (int i = 0; i < local_num; i++) {
        // Randomly generate x and y coordinates
        init_particles[i].x = random_range(NBODY_POS_MIN, NBODY_POS_MAX);
        init_particles[i].y = random_range(NBODY_POS_MIN, NBODY_POS_MAX);
        // Save the initial position to history
        history[0][i].x = init_particles[i].x;
        history[0][i].y = init_particles[i].y;

        // Randomly generate mass
        float mass = random_range(NBODY_MASS_MIN, NBODY_MASS_MAX);
        init_particles[i].mass = mass;
        // Determine the display radius based on the mass static_masses
        static_masses[i] = (uint32_t) get_radius_idx(mass);

        // Initialize vx and vy to 0
        init_particles[i].vx = 0.0f;
        init_particles[i].vy = 0.0f;
    }
    pthread_mutex_unlock(&state_mutex);

    // Write the configuration and initial bodies to the hardware
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
        // Clear read status / output pointer
        ioctl(nbody_fd, NBODY_CLEAR_READ);
        // 写 num_bodies 和 gap 到硬件
        if (ioctl(nbody_fd, NBODY_WRITE_CONFIG, &cfg) < 0)
            perror("NBODY_WRITE_CONFIG");
        // Write all initial particles to the hardware input buffer
        if (ioctl(nbody_fd, NBODY_WRITE_BODIES, &barg) < 0)
            perror("NBODY_WRITE_BODIES");
        pthread_mutex_unlock(&hw_mutex);
    }

    // Reset history state
    pthread_mutex_lock(&state_mutex);
    // history[0] stores the initial frame and the next written to history[1]
    h_head = 1;
    h_count = 1;
    view_idx = 0;
    //pthread_cond_broadcast(&frame_ready_cond);
    pthread_mutex_unlock(&state_mutex);

    // Free the memory for array
    free(init_particles);
}

// Nbody (Accelerator Simulation) Thread
void *nbody_thread(void *arg)
{
    // It is merely there to avoid a compiler warning
    (void) arg;

    // Open /dev/nbody to obtain a file descriptor
    // User-space applications communicate with it via ioctl
    nbody_fd = open("/dev/nbody", O_RDWR);
    if (nbody_fd < 0)
        perror("open /dev/nbody");

    // Regenerate initial particles and write to hardware
    reset_system();

    /*
    Check if the process is paused; if not, select the next history slot
    Initiate hardware computation and wait for completion
    Read the results from the hardware into next_idx and update the history
    */
    while (running) {
        nbody_read_arg_t rarg;

        int done = 0;
        unsigned int local_generation;

        // Copy global variables to local variables
        pthread_mutex_lock(&state_mutex);
        int paused = is_paused;
        int local_num = num_bodies;
        // Subsequently determine whether a reset occurred during this round
        local_generation = sim_generation;
        pthread_mutex_unlock(&state_mutex);

        // If paused or the hardware is not open, sleep for 10 ms
        // Then resume checking in the next iteration
        if (paused || nbody_fd < 0) {
            usleep(10000);
            continue;
        }

        // After writing to the last slot, it wraps back around to the beginning
        pthread_mutex_lock(&state_mutex);
        int next_idx = h_head & (MAX_HISTORY - 1);
        pthread_mutex_unlock(&state_mutex);

        // Have the kernel driver write REG_GO = 1 
        // Trigger the hardware to begin computation
        pthread_mutex_lock(&hw_mutex);
        if (ioctl(nbody_fd, NBODY_START_RUN) < 0) {
            perror("NBODY_START_RUN");
            pthread_mutex_unlock(&hw_mutex);
            usleep(10000);
            continue;
        }
        
        // Polling method waiting for done
        while (running && !done) {
            if (ioctl(nbody_fd, NBODY_CHECK_DONE, &done) < 0) {
                perror("NBODY_CHECK_DONE");
                break;
            }
            if (!done)
                usleep(1000);
        }

        // If process being exit
        if (!running) {
            pthread_mutex_unlock(&hw_mutex);
            break;
        }
        
        // Once the hardware operations are complete, read the results
        rarg.results = history[next_idx];
        rarg.count = (uint32_t) local_num;
        if (ioctl(nbody_fd, NBODY_READ_RESULTS, &rarg) < 0)
            perror("NBODY_READ_RESULTS");

        // Clear the read state and prepare for the next read
        if (ioctl(nbody_fd, NBODY_CLEAR_READ) < 0)
            perror("NBODY_CLEAR_READ");
        pthread_mutex_unlock(&hw_mutex);

        // Update history
        pthread_mutex_lock(&state_mutex);
        /*
        If no reset occurs during this hardware computation cycle,
        increment normally to the next frame. The old result will not be 
        updated to the history, preventing the old frame position—prior 
        to the reset—from being marked
        */
        if (local_generation == sim_generation) {
            h_head = (h_head + 1) & (MAX_HISTORY - 1);
            if (h_count < MAX_HISTORY)
                h_count++;
            view_idx = h_count - 1;
            //pthread_cond_broadcast(&frame_ready_cond);
        }
        pthread_mutex_unlock(&state_mutex);
    }

    // Clear hardware status
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

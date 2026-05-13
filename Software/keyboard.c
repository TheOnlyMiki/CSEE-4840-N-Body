#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "keyboard.h"
#include "nbody.h"
#include "usbkeyboard.h"

static struct libusb_device_handle *keyboard = NULL;
static uint8_t endpoint_address;

// Open keyboard device
int keyboard_open(void)
{
    // This function is implemented in usbkeyboard.c
    // it scans USB devices to locate a HID keyboard
    keyboard = openkeyboard(&endpoint_address);
    if (!keyboard)
        fprintf(stderr, "Warning: did not find a USB keyboard\n");

    return keyboard != NULL;
}

/*
This function returns the current monotonic time in milliseconds
It uses CLOCK_MONOTONIC rather than standard wall-clock time
The monotonic clock does not jump or shift when the system time is adjusted
More suitable for timing key-repeat intervals
*/
static unsigned long long monotonic_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ull +
           (unsigned long long)ts.tv_nsec / 1000000ull;
}

// Converting USB HID Keycodes into Program Actions
static void handle_keycode(uint8_t keycode)
{
    // SPACE - Pause/Play
    if (keycode == 0x2c) {
        nbody_toggle_pause();
    } 
    // W - Gap Increase 
    else if (keycode == 0x1a) {
        nbody_set_gap_delta(1);
    } 
    // S - Gap Decrease
    else if (keycode == 0x16) {
        nbody_set_gap_delta(-1);
    } 
    // A - Frame Backward
    else if (keycode == 0x04) {
        nbody_show_frame_delta(-1);
    } 
    // D - Frame Forward 
    else if (keycode == 0x07) {
        nbody_show_frame_delta(1);
    } 
    // R - Reset
    else if (keycode == 0x15) {
        nbody_request_reset();
    } 
    // Q - Exit
    else if (keycode == 0x14) {
        nbody_request_quit();
    }
}

/* Keyboard Thread
Read a packet from the USB keyboard, waiting for a maximum of 50 ms
If a timeout occurs: if the 'A' or 'D' key is currently being held down 
and the repeat interval has elapsed, trigger the corresponding action
Otherwise, proceed to the next iteration
If the read operation fails, proceed to the next iteration
Check if the current packet is identical to the previous packet
If no key is currently pressed, clear the repeat state and proceed to the next iteration
If the key is A/D, enable long-press repeat functionality and proceed to the next iteration
For all other keys: if the current packet is identical to the previous packet, ignore it
Otherwise, process the key press once
*/
void *keyboard_handler(void *arg)
{
    // It is merely there to avoid a compiler warning
    (void) arg;

    /*
    These two values ​​are used to scroll through historical frames when holding down A/D
    After the initial press of A/D, a 350 ms delay before continuous repetition begins
    // Once repetition starts, a frame-scrolling event is triggered every 35 ms
    */
    enum {
        REPEAT_DELAY_MS = 350,
        REPEAT_INTERVAL_MS = 35,
    };

    // Data packets read from a USB keyboard
    struct usb_keyboard_packet packet;
    uint8_t last_packet[8] = { 0 };
    uint8_t held_repeat_key = 0;
    unsigned long long next_repeat_ms = 0;

    while (running) {
        /*
        Read a keyboard packet from the USB keyboard's interrupt endpoint
        If no keyboard event occurs within 50 ms, it will time out 
        rather than blocking indefinitely
        */
        int transferred = 0;
        int rc = libusb_interrupt_transfer(keyboard, endpoint_address,
                                       (unsigned char *)&packet, sizeof(packet),
                                       &transferred, 50);
        unsigned long long now = monotonic_ms();
        
        /*
        If no new keyboard packet is received this time, but a key—specifically A/D 
        was previously being held down (repeating), then as soon as the designated 
        time interval elapses, trigger the action once again
        */
        if (rc == LIBUSB_ERROR_TIMEOUT) {
            if (held_repeat_key && now >= next_repeat_ms) {
                handle_keycode(held_repeat_key);
                next_repeat_ms = now + REPEAT_INTERVAL_MS;
            }
            continue;
        }

        //If a USB transfer error occurs, or if the number of bytes read is incorrect
        if (rc != 0 || transferred != sizeof(packet))
            continue;

        // Determines whether the current packet is identical to the previous one
        // in order to prevent a single key press from being processed repeatedly
        int same_packet = (memcmp(&packet, last_packet, sizeof(packet)) == 0);
        if (!same_packet)
            memcpy(last_packet, &packet, sizeof(packet));

        // Take only the first keycode
        uint8_t keycode = packet.keycode[0];
        if (keycode == 0) {
            held_repeat_key = 0;
            continue;
        }

        // A and D are special because they support long-press repeat
        if (keycode == 0x04 || keycode == 0x07) {
            if (!same_packet || held_repeat_key != keycode) {
                handle_keycode(keycode);
                held_repeat_key = keycode;
                next_repeat_ms = now + REPEAT_DELAY_MS;
            } else if (now >= next_repeat_ms) {
                handle_keycode(keycode);
                next_repeat_ms = now + REPEAT_INTERVAL_MS;
            }
            continue;
        }

        // If the current packet is identical to the previous one
        // it indicates that the key is still being held down
        // then do not trigger a duplicate event
        held_repeat_key = 0;
        if (same_packet)
            continue;

        handle_keycode(keycode);
    }

    return NULL;
}

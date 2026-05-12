#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "keyboard.h"
#include "nbody.h"
#include "usbkeyboard.h"

static struct libusb_device_handle *keyboard = NULL;
static uint8_t endpoint_address;

int keyboard_open(void)
{
    keyboard = openkeyboard(&endpoint_address);
    if (!keyboard)
        fprintf(stderr, "Warning: did not find a USB keyboard\n");

    return keyboard != NULL;
}

static unsigned long long monotonic_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ull +
           (unsigned long long)ts.tv_nsec / 1000000ull;
}

static void handle_keycode(uint8_t keycode)
{
    if (keycode == 0x2c) {
        nbody_toggle_pause();
    } else if (keycode == 0x1a) {
        nbody_set_gap_delta(1);
    } else if (keycode == 0x16) {
        nbody_set_gap_delta(-1);
    } else if (keycode == 0x04) {
        nbody_show_frame_delta(-1);
    } else if (keycode == 0x07) {
        nbody_show_frame_delta(1);
    } else if (keycode == 0x15) {
        nbody_request_reset();
    } else if (keycode == 0x14) {
        nbody_request_quit();
    }
}

void *keyboard_handler(void *arg)
{
    enum {
        REPEAT_DELAY_MS = 350,
        REPEAT_INTERVAL_MS = 35,
    };
    struct usb_keyboard_packet packet;
    uint8_t last_packet[8] = { 0 };
    uint8_t held_repeat_key = 0;
    unsigned long long next_repeat_ms = 0;

    (void)arg;

    while (running) {
        int transferred = 0;

        int rc = libusb_interrupt_transfer(keyboard, endpoint_address,
                                       (unsigned char *)&packet, sizeof(packet),
                                       &transferred, 50);
        unsigned long long now = monotonic_ms();

        if (rc == LIBUSB_ERROR_TIMEOUT) {
            if (held_repeat_key && now >= next_repeat_ms) {
                handle_keycode(held_repeat_key);
                next_repeat_ms = now + REPEAT_INTERVAL_MS;
            }
            continue;
        }
        if (rc != 0 || transferred != sizeof(packet))
            continue;

        int same_packet = (memcmp(&packet, last_packet, sizeof(packet)) == 0);
        if (!same_packet)
            memcpy(last_packet, &packet, sizeof(packet));

        uint8_t keycode = packet.keycode[0];
        if (keycode == 0) {
            held_repeat_key = 0;
            continue;
        }

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

        held_repeat_key = 0;
        if (same_packet)
            continue;

        handle_keycode(keycode);
    }

    return NULL;
}

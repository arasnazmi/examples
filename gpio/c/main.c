// Copyright (c) 2025 by T3 Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//     https://docs.t3gemstone.org/en/license
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <errno.h>
#include <gpiod.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#define LED_RED_BRIGHTNESS_PATH "/sys/class/leds/RED/brightness"
#define LED_GREEN_BRIGHTNESS_PATH "/sys/class/leds/GREEN/brightness"

// Global variables
static struct gpiod_chip* g_chip = NULL;
static struct gpiod_line* g_line_gpio27 = NULL; // GPIO27 set to active-high output with low value
static struct gpiod_line* g_line_gpio22 = NULL; // GPIO22 set to input with pull-up resistor enabled (normally high)
static bool g_leds_initialized = false;

static int set_led(const char* brightness_path, int value)
{
    FILE* brightness_file = fopen(brightness_path, "w");
    if (!brightness_file)
    {
        fprintf(stderr, "Failed to open %s: %s\n", brightness_path, strerror(errno));
        return -1;
    }

    if (fprintf(brightness_file, "%d\n", value ? 1 : 0) < 0)
    {
        fprintf(stderr, "Failed to write %s: %s\n", brightness_path, strerror(errno));
        fclose(brightness_file);
        return -1;
    }

    if (fclose(brightness_file) != 0)
    {
        fprintf(stderr, "Failed to close %s: %s\n", brightness_path, strerror(errno));
        return -1;
    }

    return 0;
}

void cleanup(void)
{
    if (g_leds_initialized)
    {
        set_led(LED_RED_BRIGHTNESS_PATH, 0);
        set_led(LED_GREEN_BRIGHTNESS_PATH, 0);
    }

    if (g_line_gpio27)
        gpiod_line_release(g_line_gpio27);
    if (g_line_gpio22)
        gpiod_line_release(g_line_gpio22);

    if (g_chip)
        gpiod_chip_close(g_chip);
}

void signal_handler(__attribute__((unused)) int sig)
{
    printf("\nShutting down...\n");
    exit(0);
}

void delay_ms(int ms)
{
    struct timespec request = {ms / 1000, (ms % 1000) * 1.0E6};
    struct timespec remaining;

    while (thrd_sleep(&request, &remaining) == -1 && errno == EINTR)
    {
        request = remaining; // Sleep again with remaining time if interrupted
    }
}

int main()
{
    int ret;
    int prev_input_state = 0;
    int current_input_state = 0;

    atexit(cleanup);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_chip = gpiod_chip_open_by_name("gpiochip2");
    if (!g_chip)
    {
        fprintf(stderr, "Failed to open gpiochip2\n");
        return EXIT_FAILURE;
    }

    g_line_gpio27 = gpiod_chip_get_line(g_chip, 33);
    g_line_gpio22 = gpiod_chip_get_line(g_chip, 41);

    if (!g_line_gpio27 || !g_line_gpio22)
    {
        fprintf(stderr, "Failed to get GPIO lines\n");
        return EXIT_FAILURE;
    }

    // Configure gpiochip2-33 as active-high output with value 0
    ret = gpiod_line_request_output(g_line_gpio27, "gpio_example", 0);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to configure GPIO27 as output\n");
        return EXIT_FAILURE;
    }

    // Configure gpiochip2-41 as pull-up input
    ret = gpiod_line_request_input_flags(g_line_gpio22, "gpio_example", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to configure GPIO22 as input\n");
        return EXIT_FAILURE;
    }

    if (set_led(LED_RED_BRIGHTNESS_PATH, 0) < 0 ||
        set_led(LED_GREEN_BRIGHTNESS_PATH, 0) < 0)
    {
        fprintf(stderr, "Run the example with permission to control the user LEDs.\n");
        return EXIT_FAILURE;
    }

    g_leds_initialized = true;

    printf("GPIO configuration complete:\n");
    printf("- gpiochip2-33 (GPIO27): active-high output, value=0\n");
    printf("- gpiochip2-41 (GPIO22): pull-up input\n");
    printf("- RED LED              : %s\n", LED_RED_BRIGHTNESS_PATH);
    printf("- GREEN LED            : %s\n", LED_GREEN_BRIGHTNESS_PATH);
    printf("\nWaiting for input transitions on GPIO22...\n");
    printf("Press Ctrl+C to exit\n\n");

    // Read initial state of input
    prev_input_state = gpiod_line_get_value(g_line_gpio22);
    if (prev_input_state < 0)
    {
        fprintf(stderr, "Failed to read initial input state\n");
        return EXIT_FAILURE;
    }

    while (1)
    {
        current_input_state = gpiod_line_get_value(g_line_gpio22);
        if (current_input_state < 0)
        {
            fprintf(stderr, "Failed to read input state\n");
            break;
        }

        if (prev_input_state == 1 && current_input_state == 0)
        {
            if (set_led(LED_RED_BRIGHTNESS_PATH, 1) < 0 ||
                set_led(LED_GREEN_BRIGHTNESS_PATH, 0) < 0)
            {
                break;
            }
            printf("-> Set LED_RED=HIGH, LED_GREEN=LOW\n");
        }
        else if (prev_input_state == 0 && current_input_state == 1)
        {
            if (set_led(LED_RED_BRIGHTNESS_PATH, 0) < 0 ||
                set_led(LED_GREEN_BRIGHTNESS_PATH, 1) < 0)
            {
                break;
            }
            printf("-> Set LED_RED=LOW, LED_GREEN=HIGH\n");
        }

        prev_input_state = current_input_state;

        // Small delay to avoid excessive CPU usage
        delay_ms(10);
    }

    return EXIT_SUCCESS;
}

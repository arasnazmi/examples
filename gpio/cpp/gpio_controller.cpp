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

#include "gpio_controller.h"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <threads.h>
#include <unistd.h>

namespace
{
constexpr const char* LED_RED_BRIGHTNESS_PATH = "/sys/class/leds/RED/brightness";
constexpr const char* LED_GREEN_BRIGHTNESS_PATH = "/sys/class/leds/GREEN/brightness";
} // namespace

GpioController::GpioController() {}

GpioController::~GpioController()
{
    if (m_leds_initialized)
    {
        set_led(LED_RED_BRIGHTNESS_PATH, 0);
        set_led(LED_GREEN_BRIGHTNESS_PATH, 0);
    }

    if (m_line_gpio27)
    {
        gpiod_line_release(m_line_gpio27);
        m_line_gpio27 = nullptr;
    }
    if (m_line_gpio22)
    {
        gpiod_line_release(m_line_gpio22);
        m_line_gpio22 = nullptr;
    }

    if (m_chip)
    {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
    }
}

int GpioController::set_led(const char* brightness_path, int value)
{
    std::FILE* brightness_file = std::fopen(brightness_path, "w");
    if (!brightness_file)
    {
        std::cerr << "Failed to open " << brightness_path
                  << ": " << std::strerror(errno) << std::endl;
        return 1;
    }

    if (std::fprintf(brightness_file, "%d\n", value ? 1 : 0) < 0)
    {
        std::cerr << "Failed to write " << brightness_path
                  << ": " << std::strerror(errno) << std::endl;
        std::fclose(brightness_file);
        return 1;
    }

    if (std::fclose(brightness_file) != 0)
    {
        std::cerr << "Failed to close " << brightness_path
                  << ": " << std::strerror(errno) << std::endl;
        return 1;
    }

    return 0;
}

void GpioController::delay_ms(int ms)
{
    struct timespec request = {ms / 1000, (ms % 1000) * 1'000'000};
    struct timespec remaining;

    while (thrd_sleep(&request, &remaining) == -1 && errno == EINTR)
    {
        request = remaining; // Sleep again with remaining time if interrupted
    }
}

int GpioController::initialize()
{
    m_chip = gpiod_chip_open_by_name("gpiochip2");
    if (!m_chip)
    {
        std::cerr << "Failed to open gpiochip2" << std::endl;
        return 1;
    }

    m_line_gpio27 = gpiod_chip_get_line(m_chip, 33);
    m_line_gpio22 = gpiod_chip_get_line(m_chip, 41);

    if (!m_line_gpio27 || !m_line_gpio22)
    {
        std::cerr << "Failed to get GPIO lines" << std::endl;
        return 1;
    }

    if (configure_outputs() || configure_inputs())
    {
        return 1;
    }

    if (set_led(LED_RED_BRIGHTNESS_PATH, 0) ||
        set_led(LED_GREEN_BRIGHTNESS_PATH, 0))
    {
        std::cerr << "Run the example with permission to control the user LEDs."
                  << std::endl;
        return 1;
    }

    m_leds_initialized = true;

    // Read initial state of input
    m_prev_input_state = gpiod_line_get_value(m_line_gpio22);
    if (m_prev_input_state < 0)
    {
        std::cerr << "Failed to read initial input state" << std::endl;
        return 1;
    }

    print_configuration();

    return 0;
}

int GpioController::configure_outputs()
{
    // Configure gpiochip2-33 as active-high output with value 0
    const int ret = gpiod_line_request_output(m_line_gpio27, "gpio_example", 0);
    if (ret < 0)
    {
        std::cerr << "Failed to configure GPIO27 as output" << std::endl;
        return 1;
    }

    return 0;
}

int GpioController::configure_inputs()
{
    // Configure gpiochip2-41 as pull-up input
    int ret = gpiod_line_request_input_flags(m_line_gpio22, "gpio_example", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    if (ret < 0)
    {
        std::cerr << "Failed to configure GPIO22 as input" << std::endl;
        return 1;
    }

    return 0;
}

void GpioController::print_configuration()
{
    std::cout << "GPIO configuration complete:" << std::endl;
    std::cout << "- gpiochip2-33 (GPIO27): active-high output, value=0" << std::endl;
    std::cout << "- gpiochip2-41 (GPIO22): pull-up input" << std::endl;
    std::cout << "- RED LED              : " << LED_RED_BRIGHTNESS_PATH << std::endl;
    std::cout << "- GREEN LED            : " << LED_GREEN_BRIGHTNESS_PATH << std::endl;
    std::cout << std::endl;
    std::cout << "Waiting for input transitions on GPIO22..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl << std::endl;
}

void GpioController::run()
{
    m_is_running = true;
    while (m_is_running)
    {
        m_current_input_state = gpiod_line_get_value(m_line_gpio22);
        if (m_current_input_state < 0)
        {
            std::cerr << "Failed to read input state" << std::endl;
            break;
        }

        if (m_prev_input_state == 1 && m_current_input_state == 0)
        {
            if (set_led(LED_RED_BRIGHTNESS_PATH, 1) ||
                set_led(LED_GREEN_BRIGHTNESS_PATH, 0))
            {
                break;
            }
            std::cout << "-> Set LED_RED=HIGH, LED_GREEN=LOW" << std::endl;
        }
        if (m_prev_input_state == 0 && m_current_input_state == 1)
        {
            if (set_led(LED_RED_BRIGHTNESS_PATH, 0) ||
                set_led(LED_GREEN_BRIGHTNESS_PATH, 1))
            {
                break;
            }
            std::cout << "-> Set LED_RED=LOW, LED_GREEN=HIGH" << std::endl;
        }

        m_prev_input_state = m_current_input_state;

        // Small delay to avoid excessive CPU usage
        delay_ms(10);
    }
}

void GpioController::stop()
{
    m_is_running = false;
}

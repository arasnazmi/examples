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

#ifndef GPIO_CONTROLLER_H
#define GPIO_CONTROLLER_H

#include <gpiod.h>

class GpioController
{
  public:
    GpioController();
    ~GpioController();

    GpioController(const GpioController&) = delete;
    GpioController& operator=(const GpioController&) = delete;

    int initialize();
    void run();
    void stop();

  private:
    struct gpiod_chip* m_chip {};

    struct gpiod_line* m_line_gpio27 {}; // GPIO27 set to active-high output with low value
    struct gpiod_line* m_line_gpio22 {}; // GPIO22 set to input with pull-up resistor enabled (normally high)

    bool m_leds_initialized {};
    bool m_is_running {};
    int m_prev_input_state {};
    int m_current_input_state {};

    int configure_outputs();
    int configure_inputs();
    static int set_led(const char* brightness_path, int value);
    void print_configuration();
    static void delay_ms(int ms);
};

#endif // GPIO_CONTROLLER_H

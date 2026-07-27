# Copyright (c) 2025 by T3 Foundation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#     https://docs.t3gemstone.org/en/license
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import sys
import time
from typing import Optional

import gpiod


LED_RED_BRIGHTNESS_PATH = "/sys/class/leds/RED/brightness"
LED_GREEN_BRIGHTNESS_PATH = "/sys/class/leds/GREEN/brightness"


class GpioController:
    def __init__(self):
        self.m_is_running: bool = False
        self.m_chip: Optional[gpiod.Chip] = None

        self.m_output_request: Optional[gpiod.LineRequest] = None
        self.m_input_request: Optional[gpiod.LineRequest] = None

        self.m_line_gpio27: int = 33  # GPIO27 set to active-high output with low value
        self.m_line_gpio22: int = 41  # GPIO22 set to input with pull-up resistor enabled (normally high)

        self.m_leds_initialized: bool = False
        self.m_prev_input_state: int = 0
        self.m_current_input_state: int = 0

        self._cleaned_up: bool = False

    def __del__(self):
        self.cleanup()

    @staticmethod
    def _set_led(brightness_path: str, value: int) -> int:
        try:
            with open(
                brightness_path,
                "w",
                encoding="ascii",
            ) as brightness_file:
                brightness_file.write(f"{1 if value else 0}\n")
        except OSError as error:
            print(
                f"Failed to write {brightness_path}: {error}",
                file=sys.stderr,
            )
            return 1

        return 0

    def initialize(self) -> int:
        try:
            self.m_chip = gpiod.Chip("/dev/gpiochip2")

            output_config = {
                self.m_line_gpio27: gpiod.LineSettings(
                    direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE
                ),
            }

            self.m_output_request = self.m_chip.request_lines(consumer="gpio_example", config=output_config)

            input_config = {
                self.m_line_gpio22: gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT, bias=gpiod.line.Bias.PULL_UP
                )
            }

            self.m_input_request = self.m_chip.request_lines(consumer="gpio_example", config=input_config)

        except Exception as e:
            print(f"Failed to initialize GPIO: {e}", file=sys.stderr)
            return 1

        if (
            self._set_led(LED_RED_BRIGHTNESS_PATH, 0)
            or self._set_led(LED_GREEN_BRIGHTNESS_PATH, 0)
        ):
            print(
                "Run the example with permission to control the user LEDs.",
                file=sys.stderr,
            )
            return 1

        self.m_leds_initialized = True

        # Read initial state of input
        try:
            self.m_prev_input_state = self.m_input_request.get_value(self.m_line_gpio22).value
        except Exception as e:
            print(f"Failed to read initial input state: {e}", file=sys.stderr)
            return 1

        self._print_configuration()
        return 0

    def _print_configuration(self) -> None:
        print("GPIO configuration complete:")
        print("- gpiochip2-33 (GPIO27): active-high output, value=0")
        print("- gpiochip2-41 (GPIO22): pull-up input")
        print(f"- RED LED              : {LED_RED_BRIGHTNESS_PATH}")
        print(f"- GREEN LED            : {LED_GREEN_BRIGHTNESS_PATH}")
        print()
        print("Waiting for input transitions on GPIO22...")
        print("Press Ctrl+C to exit")
        print()

    def run(self) -> None:
        try:
            self.m_is_running = True
            while self.m_is_running:
                try:
                    self.m_current_input_state = self.m_input_request.get_value(self.m_line_gpio22).value
                except Exception as e:
                    print(f"Failed to read input state: {e}", file=sys.stderr)
                    break

                if self.m_prev_input_state == 1 and self.m_current_input_state == 0:
                    if (
                        self._set_led(LED_RED_BRIGHTNESS_PATH, 1)
                        or self._set_led(LED_GREEN_BRIGHTNESS_PATH, 0)
                    ):
                        break
                    print("-> Set LED_RED=HIGH, LED_GREEN=LOW")
                elif self.m_prev_input_state == 0 and self.m_current_input_state == 1:
                    if (
                        self._set_led(LED_RED_BRIGHTNESS_PATH, 0)
                        or self._set_led(LED_GREEN_BRIGHTNESS_PATH, 1)
                    ):
                        break
                    print("-> Set LED_RED=LOW, LED_GREEN=HIGH")

                self.m_prev_input_state = self.m_current_input_state

                # Small delay to avoid excessive CPU usage
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nReceived interrupt signal")
        except Exception as e:
            print(f"Error in main loop: {e}", file=sys.stderr)

    def stop(self) -> None:
        self.m_is_running = False

    def cleanup(self) -> None:
        if self._cleaned_up:
            return

        self._cleaned_up = True

        if self.m_leds_initialized:
            self._set_led(LED_RED_BRIGHTNESS_PATH, 0)
            self._set_led(LED_GREEN_BRIGHTNESS_PATH, 0)
            self.m_leds_initialized = False

        if self.m_output_request:
            self.m_output_request.release()
            self.m_output_request = None

        if self.m_input_request:
            self.m_input_request.release()
            self.m_input_request = None

        if self.m_chip:
            self.m_chip.close()
            self.m_chip = None

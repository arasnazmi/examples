#!.venv/bin/python3

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

import signal
import sys

from gpio_controller import GpioController

# Global variables
g_gpio_controller = GpioController()


def signal_handler(sig, frame):
    global g_gpio_controller

    print("\nShutting down...")
    g_gpio_controller.stop()


def main():
    global g_gpio_controller

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if g_gpio_controller.initialize():
        print("Failed to initialize GPIO controller", file=sys.stderr)
        return 1

    try:
        g_gpio_controller.run()
    finally:
        g_gpio_controller.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())

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

// ICM-20948 9-DOF orientation tracking.
//
// Roll and pitch can be recovered from the accelerometer alone, because gravity
// is an absolute reference that never drifts. Yaw cannot: with only an
// accelerometer and a gyroscope there is nothing to correct the yaw estimate
// against, so the gyro's small zero-rate offset integrates without bound and
// the heading walks away at a steady rate.
//
// The magnetometer supplies the missing reference. This example brings up the
// AK09916 on the ICM-20948's auxiliary I2C bus, calibrates it for the magnetic
// distortion caused by the board itself, and fuses all nine axes in a Mahony
// filter whose integral term also learns and cancels the gyro bias.

#include "ahrs.h"
#include "icm20948.h"
#include "mag_cal.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SPI_DEV_PATH_DEFAULT "/dev/spidev0.3"
#define MAG_CAL_PATH_DEFAULT "imu_mag_cal.conf"

#define LOOP_HZ 100
#define LOOP_PERIOD_NS (1000000000L / LOOP_HZ)
#define PRINT_HZ 10

// Sampled at roughly the output data rate: polling faster than the sensor
// updates just re-reads the same value, which adds time without adding
// averaging.
#define GYRO_CAL_SAMPLES 500
#define GYRO_CAL_INTERVAL_US 10000

#define MAG_CAL_CAPACITY 3000
#define MAG_CAL_MIN_DELTA_UT 1.0f
#define MAG_CAL_TIMEOUT_S 60.0f
#define MAG_CAL_GOOD_RESIDUAL 0.05f

#define DEG_TO_RAD 0.01745329252f
#define NSEC_PER_SEC 1000000000L

static volatile sig_atomic_t g_is_running = 1;

typedef struct
{
    const char* spi_dev_path;
    const char* mag_cal_path;
    float declination;
    bool force_mag_cal;
    bool skip_mag_cal;
    bool disable_mag;
    bool require_mag;
    bool mag_debug;
} options_t;

static void signal_handler(__attribute__((unused)) int sig)
{
    g_is_running = 0;
}

static void timespec_add_ns(struct timespec* ts, long nsec)
{
    ts->tv_nsec += nsec;
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

static double timespec_diff_s(const struct timespec* end, const struct timespec* start)
{
    return (double)(end->tv_sec - start->tv_sec) + (double)(end->tv_nsec - start->tv_nsec) / 1.0e9;
}

static void print_usage(const char* program)
{
    printf("Usage: %s [options]\n\n", program);
    printf("  -d, --device PATH       SPI device (default %s)\n", SPI_DEV_PATH_DEFAULT);
    printf("  -c, --cal-file PATH     Magnetometer calibration file (default %s)\n", MAG_CAL_PATH_DEFAULT);
    printf("  -m, --declination DEG   Magnetic declination, East positive, for true North\n");
    printf("      --calibrate         Force magnetometer calibration even if a file exists\n");
    printf("      --no-calibrate      Run uncalibrated (yaw still referenced, heading biased)\n");
    printf("      --no-mag            Disable the magnetometer (diagnostic: yaw will drift)\n");
    printf("      --require-mag       Fail instead of continuing if the magnetometer is absent\n");
    printf("      --mag-debug         Probe every auxiliary bus arrangement if bring-up fails\n");
    printf("  -h, --help              Show this message\n");
}

static int parse_options(int argc, char** argv, options_t* opts)
{
    opts->spi_dev_path = SPI_DEV_PATH_DEFAULT;
    opts->mag_cal_path = MAG_CAL_PATH_DEFAULT;
    opts->declination = 0.0f;
    opts->force_mag_cal = false;
    opts->skip_mag_cal = false;
    opts->disable_mag = false;
    opts->require_mag = false;
    opts->mag_debug = false;

    for (int i = 1; i < argc; i++)
    {
        const char* arg = argv[i];
        const bool has_value = (i + 1) < argc;

        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0)
        {
            print_usage(argv[0]);
            return 1;
        }
        else if ((strcmp(arg, "-d") == 0 || strcmp(arg, "--device") == 0) && has_value)
        {
            opts->spi_dev_path = argv[++i];
        }
        else if ((strcmp(arg, "-c") == 0 || strcmp(arg, "--cal-file") == 0) && has_value)
        {
            opts->mag_cal_path = argv[++i];
        }
        else if ((strcmp(arg, "-m") == 0 || strcmp(arg, "--declination") == 0) && has_value)
        {
            opts->declination = strtof(argv[++i], NULL);
        }
        else if (strcmp(arg, "--calibrate") == 0)
        {
            opts->force_mag_cal = true;
        }
        else if (strcmp(arg, "--no-calibrate") == 0)
        {
            opts->skip_mag_cal = true;
        }
        else if (strcmp(arg, "--no-mag") == 0)
        {
            opts->disable_mag = true;
        }
        else if (strcmp(arg, "--require-mag") == 0)
        {
            opts->require_mag = true;
        }
        else if (strcmp(arg, "--mag-debug") == 0)
        {
            opts->mag_debug = true;
        }
        else
        {
            fprintf(stderr, "Unrecognised argument: %s\n\n", arg);
            print_usage(argv[0]);
            return -1;
        }
    }

    return 0;
}

// Average the gyroscope at rest to find its zero-rate offset. Every gyro reads
// slightly non-zero when stationary, and integrating that offset is precisely
// what becomes angle drift.
static int run_gyro_calibration(icm20948_handle_t icm20948, const icm20948_data_t* data)
{
    printf("Calibrating gyroscope. Keep the board completely still...\n");
    fflush(stdout);

    if (icm20948_calibrate_gyro(icm20948, GYRO_CAL_SAMPLES, GYRO_CAL_INTERVAL_US) != 0)
    {
        return -1;
    }

    printf("  Gyro bias: x=%+.3f y=%+.3f z=%+.3f deg/s\n", (double)data->gx_bias, (double)data->gy_bias,
           (double)data->gz_bias);
    return 0;
}

// Sample the magnetometer while the operator turns the board through as many
// orientations as possible, then fit a sphere to the result.
static int run_mag_calibration(icm20948_handle_t icm20948, const icm20948_data_t* data, mag_cal_t* cal,
                               const char* path)
{
    float* storage = malloc(sizeof(float) * 3 * MAG_CAL_CAPACITY);
    if (storage == NULL)
    {
        fprintf(stderr, "Failed to allocate magnetometer calibration buffer\n");
        return -1;
    }

    mag_cal_collector_t collector;
    mag_cal_collector_init(&collector, storage, MAG_CAL_CAPACITY, MAG_CAL_MIN_DELTA_UT);

    printf("\nMagnetometer calibration\n");
    printf("------------------------\n");
    printf("Slowly rotate the board through every orientation you can -- turn it\n");
    printf("over, stand it on each edge, trace a figure eight in the air. Keep it\n");
    printf("away from metal, motors, speakers and mains cabling.\n\n");

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (g_is_running)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const double elapsed = timespec_diff_s(&now, &start);
        if (elapsed > (double)MAG_CAL_TIMEOUT_S)
        {
            break;
        }

        if (icm20948_get_agmt(icm20948) != 0)
        {
            free(storage);
            return -1;
        }
        if (data->mag_valid)
        {
            mag_cal_collector_add(&collector, data->mx, data->my, data->mz);
        }

        const float coverage = mag_cal_collector_coverage(&collector);
        printf("\r  %5.1fs  samples: %4zu  coverage: %3.0f%%   ", elapsed, collector.count, (double)coverage * 100.0);
        fflush(stdout);

        // Stop early once the sphere is well covered and there is plenty of data.
        if (coverage > 0.99f && collector.count > MAG_CAL_CAPACITY / 2)
        {
            break;
        }

        usleep(10000);
    }
    printf("\n");

    if (!g_is_running)
    {
        free(storage);
        return -1;
    }

    if (mag_cal_solve(&collector, cal) != 0)
    {
        fprintf(stderr, "Calibration fit failed. Collected %zu samples covering %.0f%% of the sphere;\n",
                collector.count, (double)mag_cal_collector_coverage(&collector) * 100.0);
        fprintf(stderr, "try again and rotate through more orientations.\n");
        free(storage);
        return -1;
    }

    const float residual = mag_cal_residual(&collector, cal);
    printf("  Hard-iron offset: x=%+.2f y=%+.2f z=%+.2f uT\n", (double)cal->offset[0], (double)cal->offset[1],
           (double)cal->offset[2]);
    printf("  Soft-iron scale:  x=%.3f y=%.3f z=%.3f\n", (double)cal->scale[0], (double)cal->scale[1],
           (double)cal->scale[2]);
    printf("  Field strength:   %.1f uT (Earth's field is 25-65 uT)\n", (double)cal->radius);
    printf("  Fit residual:     %.1f%%\n", (double)residual * 100.0);

    if (residual > MAG_CAL_GOOD_RESIDUAL)
    {
        printf("  Warning: residual above %.0f%%. The board was probably near something\n",
               (double)MAG_CAL_GOOD_RESIDUAL * 100.0);
        printf("  magnetic during calibration. Consider redoing it somewhere clearer.\n");
    }

    if (mag_cal_save(cal, path) != 0)
    {
        fprintf(stderr, "  Warning: could not write calibration to %s\n", path);
    }
    else
    {
        printf("  Saved to %s\n", path);
    }

    free(storage);
    return 0;
}

int main(int argc, char** argv)
{
    options_t opts;
    const int parse_result = parse_options(argc, argv, &opts);
    if (parse_result != 0)
    {
        return parse_result > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    icm20948_data_t icm20948_data;
    char dev_name[] = "icm20948";
    icm20948_handle_t icm20948 = icm20948_create(&icm20948_data, dev_name);
    if (icm20948 == NULL)
    {
        return EXIT_FAILURE;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("ICM-20948 9-DOF Orientation Test\n");
    printf("================================\n\n");

    if (icm20948_spi_bus_init(icm20948, opts.spi_dev_path) != 0)
    {
        fprintf(stderr, "Failed to initialize ICM-20948 IMU with SPI bus, %s\n", opts.spi_dev_path);
        icm20948_delete(icm20948);
        return EXIT_FAILURE;
    }

    icm20948_config_t config;
    icm20948_config_default(&config);
    config.enable_mag = !opts.disable_mag;
    config.mag_required = opts.require_mag;
    config.mag_debug = opts.mag_debug;

    if (icm20948_configure(icm20948, &config) != 0)
    {
        fprintf(stderr, "Failed to configure ICM-20948 IMU\n");
        if (config.mag_required)
        {
            fprintf(stderr, "Drop --require-mag to run without the magnetometer.\n");
        }
        icm20948_delete(icm20948);
        return EXIT_FAILURE;
    }

    printf("\n");
    printf("Accel Sensitivity:      %.0f LSB/g\n", (double)icm20948_get_acce_sensitivity(icm20948));
    printf("Gyro Sensitivity:       %.1f LSB/(deg/s)\n", (double)icm20948_get_gyro_sensitivity(icm20948));
    printf("Output Data Rate:       %.1f Hz\n", (double)(ICM20948_ODR_BASE_HZ / (1.0f + config.sample_rate_div)));
    const bool mag_active = icm20948_mag_available(icm20948);
    printf("Magnetometer:           %s\n\n",
           mag_active ? "enabled, 100 Hz" : (config.enable_mag ? "unavailable" : "disabled"));

    if (run_gyro_calibration(icm20948, &icm20948_data) != 0)
    {
        icm20948_delete(icm20948);
        return EXIT_FAILURE;
    }

    mag_cal_t mag_cal;
    mag_cal_identity(&mag_cal);
    bool mag_cal_ready = false;

    if (mag_active && opts.skip_mag_cal)
    {
        printf("Skipping magnetometer calibration (--no-calibrate).\n");
        printf("  Yaw still has an absolute reference so it will not drift, but any\n");
        printf("  hard-iron offset on the board shows up as a heading error.\n");
    }
    else if (mag_active)
    {
        if (!opts.force_mag_cal && mag_cal_load(&mag_cal, opts.mag_cal_path) == 0)
        {
            printf("Loaded magnetometer calibration from %s\n", opts.mag_cal_path);
            printf("  offset: x=%+.2f y=%+.2f z=%+.2f uT, field %.1f uT\n", (double)mag_cal.offset[0],
                   (double)mag_cal.offset[1], (double)mag_cal.offset[2], (double)mag_cal.radius);
            printf("  Re-run with --calibrate to redo it.\n");
            mag_cal_ready = true;
        }
        else if (run_mag_calibration(icm20948, &icm20948_data, &mag_cal, opts.mag_cal_path) == 0)
        {
            mag_cal_ready = true;
        }
        else if (g_is_running)
        {
            fprintf(stderr, "Continuing without magnetometer calibration; heading will be unreliable.\n");
            mag_cal_identity(&mag_cal);
        }
    }

    if (!g_is_running)
    {
        icm20948_delete(icm20948);
        return EXIT_SUCCESS;
    }

    ahrs_t ahrs;
    ahrs_init(&ahrs, AHRS_DEFAULT_KP, AHRS_DEFAULT_KI);

    printf("\nStreaming. Press Ctrl+C to exit.\n");
    if (mag_active && !mag_cal_ready)
    {
        printf("(Magnetometer uncalibrated: yaw is referenced but biased.)\n");
    }
    else if (!mag_active)
    {
        printf("(No magnetometer: yaw has no absolute reference and WILL drift.)\n");
    }
    printf("\n");

    struct timespec deadline;
    struct timespec previous;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    previous = deadline;

    unsigned long iteration = 0;
    const unsigned long print_every = LOOP_HZ / PRINT_HZ;

    while (g_is_running)
    {
        if (icm20948_get_agmt(icm20948) != 0)
        {
            // A single failed transfer is not fatal; check whether the part is
            // still alive and rebuild its configuration if it has dropped off.
            if (icm20948_check_online(icm20948) != 0)
            {
                fprintf(stderr, "\nDevice lost, reconfiguring...\n");
                if (icm20948_configure(icm20948, &config) != 0)
                {
                    fprintf(stderr, "Reconfiguration failed, giving up.\n");
                    break;
                }
                ahrs_init(&ahrs, AHRS_DEFAULT_KP, AHRS_DEFAULT_KI);
            }
            timespec_add_ns(&deadline, LOOP_PERIOD_NS);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
            continue;
        }

        // Measure the real elapsed time rather than assuming the nominal loop
        // period. Bus transfers and scheduling jitter make the actual interval
        // longer than the requested one, and integrating rate against a dt that
        // is even a few percent wrong scales every rotation by that error.
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const float dt = (float)timespec_diff_s(&now, &previous);
        previous = now;

        float mag[3] = {0.0f, 0.0f, 0.0f};
        if (mag_active)
        {
            const float raw[3] = {icm20948_data.mx, icm20948_data.my, icm20948_data.mz};
            mag_cal_apply(&mag_cal, raw, mag);
        }

        ahrs_update(&ahrs, icm20948_data.gx * DEG_TO_RAD, icm20948_data.gy * DEG_TO_RAD, icm20948_data.gz * DEG_TO_RAD,
                    icm20948_data.ax, icm20948_data.ay, icm20948_data.az, mag[0], mag[1], mag[2], dt);

        if ((iteration % print_every) == 0)
        {
            float roll, pitch, yaw;
            ahrs_get_euler(&ahrs, &roll, &pitch, &yaw);
            const float heading = ahrs_get_heading(&ahrs, opts.declination);

            float bx, by, bz;
            ahrs_get_gyro_bias(&ahrs, &bx, &by, &bz);

            printf("\rRoll %+7.2f  Pitch %+7.2f  Yaw %+7.2f  Heading %6.2f  |  residual bias z %+6.3f deg/s  %.1fC  ",
                   (double)roll, (double)pitch, (double)yaw, (double)heading, (double)(bz / DEG_TO_RAD),
                   (double)icm20948_data.temp);
            fflush(stdout);
        }
        iteration++;

        timespec_add_ns(&deadline, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    }

    printf("\nShutting down...\n");
    icm20948_delete(icm20948);

    return EXIT_SUCCESS;
}

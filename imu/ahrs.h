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

#ifndef __AHRS_H__
#define __AHRS_H__

#include <stdbool.h>

// Mahony explicit complementary filter, 9-DOF (gyro + accel + magnetometer).
//
// Why this and not three independent scalar Kalman filters on Euler angles:
//
//   * A gyroscope alone can only ever *integrate* angular rate, so any residual
//     bias turns into an angle error that grows without bound. Roll and pitch
//     escape this because gravity gives them an absolute reference. Yaw has no
//     such reference unless the magnetometer supplies one -- which is exactly
//     why an accel+gyro-only estimator drifts in yaw and nowhere else.
//
//   * The filter carries orientation as a quaternion, so it stays well behaved
//     through gimbal lock (pitch = +/-90 deg) where an Euler formulation breaks.
//
//   * The integral feedback term is a running estimate of gyro bias. It is the
//     mechanism that actually removes drift: the accel/mag error signal is fed
//     back into the rate input, so a constant bias is learned and cancelled
//     rather than integrated. See `ahrs_get_gyro_bias`.
//
// World frame is NWU: +X magnetic North, +Y West, +Z Up. This matches the
// ICM-20948 accelerometer, which reads +1g on Z when the board is level.

typedef struct
{
    float q0, q1, q2, q3;                  /*!< Orientation quaternion (body -> world) */
    float integral_fb_x;                   /*!< Integral feedback == gyro bias estimate, rad/s */
    float integral_fb_y;                   /*!< Integral feedback == gyro bias estimate, rad/s */
    float integral_fb_z;                   /*!< Integral feedback == gyro bias estimate, rad/s */
    float two_kp;                          /*!< 2 * proportional gain */
    float two_ki;                          /*!< 2 * integral gain */
    bool initialised;                      /*!< Set once the quaternion has been seeded */
} ahrs_t;

#define AHRS_DEFAULT_KP 2.0f  /*!< Proportional gain: how hard accel/mag pull the estimate */
#define AHRS_DEFAULT_KI 0.05f /*!< Integral gain: how fast gyro bias is learned */

/**
 * @brief Initialise the filter with the given gains
 *
 * The quaternion is left unseeded; the first `ahrs_update` call with valid
 * accelerometer and magnetometer data snaps it directly to the measured
 * orientation so the filter starts converged instead of sweeping in from
 * an arbitrary attitude.
 *
 * @param ahrs filter object
 * @param kp   proportional gain, use AHRS_DEFAULT_KP if unsure
 * @param ki   integral gain, use AHRS_DEFAULT_KI if unsure
 */
void ahrs_init(ahrs_t* ahrs, float kp, float ki);

/**
 * @brief Advance the filter by one sample
 *
 * Accelerometer and magnetometer vectors are normalised internally, so their
 * units are irrelevant as long as each triple is self-consistent. Samples whose
 * magnitude is zero (or non-finite) are ignored, and the step degrades to pure
 * gyro integration for that iteration.
 *
 * @param ahrs filter object
 * @param gx   angular rate about X, rad/s
 * @param gy   angular rate about Y, rad/s
 * @param gz   angular rate about Z, rad/s
 * @param ax   acceleration along X, any unit
 * @param ay   acceleration along Y, any unit
 * @param az   acceleration along Z, any unit
 * @param mx   magnetic field along X, any unit, in the accel/gyro frame
 * @param my   magnetic field along Y, any unit, in the accel/gyro frame
 * @param mz   magnetic field along Z, any unit, in the accel/gyro frame
 * @param dt   time since the previous update, seconds
 */
void ahrs_update(ahrs_t* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                 float dt);

/**
 * @brief Convert the current quaternion to Tait-Bryan angles
 *
 * @param ahrs  filter object
 * @param roll  rotation about X, degrees, right-hand positive
 * @param pitch rotation about Y, degrees, right-hand positive
 * @param yaw   rotation about Z, degrees, counter-clockwise from magnetic North
 */
void ahrs_get_euler(const ahrs_t* ahrs, float* roll, float* pitch, float* yaw);

/**
 * @brief Compass heading: degrees clockwise from North, wrapped to [0, 360)
 *
 * @param ahrs        filter object
 * @param declination local magnetic declination in degrees, East positive.
 *                    Pass 0 for magnetic North, or look the value up at
 *                    https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 *                    to get true North.
 *
 * @return heading in degrees
 */
float ahrs_get_heading(const ahrs_t* ahrs, float declination);

/**
 * @brief Read back the online gyro bias estimate
 *
 * Useful as a health indicator: once converged these settle to small, steady
 * values. A component that keeps growing means the magnetometer calibration or
 * the axis mapping is wrong.
 *
 * @param ahrs filter object
 * @param bx   bias about X, rad/s
 * @param by   bias about Y, rad/s
 * @param bz   bias about Z, rad/s
 */
void ahrs_get_gyro_bias(const ahrs_t* ahrs, float* bx, float* by, float* bz);

#endif // !__AHRS_H__

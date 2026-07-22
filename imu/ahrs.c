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

#include "ahrs.h"
#include <math.h>

#define AHRS_RAD_TO_DEG 57.29577951f
#define AHRS_EPSILON 1e-9f

// Normalise a 3-vector in place. Returns false if it is degenerate, in which
// case the caller must treat the sample as unusable rather than divide by it.
static bool ahrs_normalise(float* x, float* y, float* z)
{
    const float norm_sq = (*x) * (*x) + (*y) * (*y) + (*z) * (*z);
    if (!isfinite(norm_sq) || norm_sq < AHRS_EPSILON)
    {
        return false;
    }

    const float recip = 1.0f / sqrtf(norm_sq);
    *x *= recip;
    *y *= recip;
    *z *= recip;
    return true;
}

// Seed the quaternion directly from one accel + mag sample (TRIAD).
//
// Gravity fixes two of the three degrees of freedom and the magnetic field
// fixes the third, so a single sample already determines a full orientation.
// Starting there means the filter is converged on its very first iteration
// instead of visibly swinging into place over the first few seconds.
static void ahrs_seed_from_measurement(ahrs_t* ahrs, float ax, float ay, float az, float mx, float my, float mz)
{
    // World axes expressed in the body frame. The accelerometer reads +1g along
    // Up when level, so the normalised accel vector *is* Up in body coordinates.
    float up_x = ax, up_y = ay, up_z = az;
    if (!ahrs_normalise(&up_x, &up_y, &up_z))
    {
        return;
    }

    // West = Up x M. The magnetic field points North and (in the northern
    // hemisphere) downward, but only its horizontal part survives the cross
    // product, so this is inherently tilt-compensated.
    float west_x = up_y * mz - up_z * my;
    float west_y = up_z * mx - up_x * mz;
    float west_z = up_x * my - up_y * mx;
    if (!ahrs_normalise(&west_x, &west_y, &west_z))
    {
        // Field is parallel to gravity (or absent): heading is unobservable.
        return;
    }

    // North = West x Up, orthogonal to both by construction.
    const float north_x = west_y * up_z - west_z * up_y;
    const float north_y = west_z * up_x - west_x * up_z;
    const float north_z = west_x * up_y - west_y * up_x;

    // Rows of the body->world rotation matrix are the world axes in body
    // coordinates, so R is assembled directly from the three vectors above.
    const float m00 = north_x, m01 = north_y, m02 = north_z;
    const float m10 = west_x, m11 = west_y, m12 = west_z;
    const float m20 = up_x, m21 = up_y, m22 = up_z;

    // Shepperd's method: pick the branch with the largest divisor so the
    // square root never operates on a near-zero quantity.
    const float trace = m00 + m11 + m22;
    if (trace > 0.0f)
    {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        ahrs->q0 = 0.25f * s;
        ahrs->q1 = (m21 - m12) / s;
        ahrs->q2 = (m02 - m20) / s;
        ahrs->q3 = (m10 - m01) / s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        ahrs->q0 = (m21 - m12) / s;
        ahrs->q1 = 0.25f * s;
        ahrs->q2 = (m01 + m10) / s;
        ahrs->q3 = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        const float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        ahrs->q0 = (m02 - m20) / s;
        ahrs->q1 = (m01 + m10) / s;
        ahrs->q2 = 0.25f * s;
        ahrs->q3 = (m12 + m21) / s;
    }
    else
    {
        const float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        ahrs->q0 = (m10 - m01) / s;
        ahrs->q1 = (m02 + m20) / s;
        ahrs->q2 = (m12 + m21) / s;
        ahrs->q3 = 0.25f * s;
    }

    ahrs->initialised = true;
}

void ahrs_init(ahrs_t* ahrs, float kp, float ki)
{
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->integral_fb_x = 0.0f;
    ahrs->integral_fb_y = 0.0f;
    ahrs->integral_fb_z = 0.0f;
    ahrs->two_kp = 2.0f * kp;
    ahrs->two_ki = 2.0f * ki;
    ahrs->initialised = false;
}

void ahrs_update(ahrs_t* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                 float dt)
{
    if (!isfinite(dt) || dt <= 0.0f)
    {
        return;
    }

    float acc_x = ax, acc_y = ay, acc_z = az;
    float mag_x = mx, mag_y = my, mag_z = mz;
    const bool acc_valid = ahrs_normalise(&acc_x, &acc_y, &acc_z);
    const bool mag_valid = ahrs_normalise(&mag_x, &mag_y, &mag_z);

    if (!ahrs->initialised)
    {
        if (acc_valid && mag_valid)
        {
            ahrs_seed_from_measurement(ahrs, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);
        }
        // Nothing to correct against yet, so skip straight to integration.
    }

    // Both reference vectors are needed for a full 9-DOF correction. Without
    // the magnetometer the yaw error is unobservable and feeding a bogus
    // correction in would be worse than feeding none, so this step is skipped
    // and the sample degrades to gyro-only integration.
    if (acc_valid && mag_valid && ahrs->initialised)
    {
        const float q0q0 = ahrs->q0 * ahrs->q0;
        const float q0q1 = ahrs->q0 * ahrs->q1;
        const float q0q2 = ahrs->q0 * ahrs->q2;
        const float q0q3 = ahrs->q0 * ahrs->q3;
        const float q1q1 = ahrs->q1 * ahrs->q1;
        const float q1q2 = ahrs->q1 * ahrs->q2;
        const float q1q3 = ahrs->q1 * ahrs->q3;
        const float q2q2 = ahrs->q2 * ahrs->q2;
        const float q2q3 = ahrs->q2 * ahrs->q3;
        const float q3q3 = ahrs->q3 * ahrs->q3;

        // Rotate the measured field into the world frame, then collapse it onto
        // the (horizontal, vertical) reference the estimate should agree with.
        // Only the horizontal magnitude is kept, which is what makes the yaw
        // correction independent of magnetic dip.
        const float hx = 2.0f * (mag_x * (0.5f - q2q2 - q3q3) + mag_y * (q1q2 - q0q3) + mag_z * (q1q3 + q0q2));
        const float hy = 2.0f * (mag_x * (q1q2 + q0q3) + mag_y * (0.5f - q1q1 - q3q3) + mag_z * (q2q3 - q0q1));
        const float bx = sqrtf(hx * hx + hy * hy);
        const float bz = 2.0f * (mag_x * (q1q3 - q0q2) + mag_y * (q2q3 + q0q1) + mag_z * (0.5f - q1q1 - q2q2));

        // Where the estimate says gravity (v) and the field (w) should point,
        // in the body frame. Halved to fold the factor of 2 into the gains.
        const float half_vx = q1q3 - q0q2;
        const float half_vy = q0q1 + q2q3;
        const float half_vz = q0q0 - 0.5f + q3q3;
        const float half_wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        const float half_wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        const float half_wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Orientation error as the cross product between measured and predicted
        // directions: zero when they align, and pointing along the rotation
        // axis that would bring them together otherwise.
        const float half_ex = (acc_y * half_vz - acc_z * half_vy) + (mag_y * half_wz - mag_z * half_wy);
        const float half_ey = (acc_z * half_vx - acc_x * half_vz) + (mag_z * half_wx - mag_x * half_wz);
        const float half_ez = (acc_x * half_vy - acc_y * half_vx) + (mag_x * half_wy - mag_y * half_wx);

        // Integral term. This is the part that kills drift: a constant gyro
        // bias produces a persistent error, which accumulates here until it
        // exactly cancels the bias at the rate input below.
        if (ahrs->two_ki > 0.0f)
        {
            ahrs->integral_fb_x += ahrs->two_ki * half_ex * dt;
            ahrs->integral_fb_y += ahrs->two_ki * half_ey * dt;
            ahrs->integral_fb_z += ahrs->two_ki * half_ez * dt;

            gx += ahrs->integral_fb_x;
            gy += ahrs->integral_fb_y;
            gz += ahrs->integral_fb_z;
        }

        // Proportional term: pulls the estimate toward the measurement now.
        gx += ahrs->two_kp * half_ex;
        gy += ahrs->two_kp * half_ey;
        gz += ahrs->two_kp * half_ez;
    }

    // Integrate the (corrected) rate into the quaternion.
    const float half_dt = 0.5f * dt;
    const float dgx = gx * half_dt;
    const float dgy = gy * half_dt;
    const float dgz = gz * half_dt;

    const float qa = ahrs->q0;
    const float qb = ahrs->q1;
    const float qc = ahrs->q2;
    const float qd = ahrs->q3;

    ahrs->q0 += (-qb * dgx - qc * dgy - qd * dgz);
    ahrs->q1 += (qa * dgx + qc * dgz - qd * dgy);
    ahrs->q2 += (qa * dgy - qb * dgz + qd * dgx);
    ahrs->q3 += (qa * dgz + qb * dgy - qc * dgx);

    const float norm_sq = ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3;
    if (isfinite(norm_sq) && norm_sq > AHRS_EPSILON)
    {
        const float recip = 1.0f / sqrtf(norm_sq);
        ahrs->q0 *= recip;
        ahrs->q1 *= recip;
        ahrs->q2 *= recip;
        ahrs->q3 *= recip;
    }
    else
    {
        // Numerically unrecoverable; fall back to identity and re-seed.
        ahrs->q0 = 1.0f;
        ahrs->q1 = 0.0f;
        ahrs->q2 = 0.0f;
        ahrs->q3 = 0.0f;
        ahrs->initialised = false;
    }
}

void ahrs_get_euler(const ahrs_t* ahrs, float* roll, float* pitch, float* yaw)
{
    const float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;

    *roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * AHRS_RAD_TO_DEG;

    // Clamp before asinf: rounding can push the argument just past +/-1.
    float sin_pitch = -2.0f * (q1 * q3 - q0 * q2);
    if (sin_pitch > 1.0f)
        sin_pitch = 1.0f;
    else if (sin_pitch < -1.0f)
        sin_pitch = -1.0f;
    *pitch = asinf(sin_pitch) * AHRS_RAD_TO_DEG;

    *yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * AHRS_RAD_TO_DEG;
}

float ahrs_get_heading(const ahrs_t* ahrs, float declination)
{
    float roll, pitch, yaw;
    ahrs_get_euler(ahrs, &roll, &pitch, &yaw);

    // Yaw grows counter-clockwise from North; compass headings grow clockwise.
    float heading = -(yaw + declination);
    heading = fmodf(heading, 360.0f);
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }
    return heading;
}

void ahrs_get_gyro_bias(const ahrs_t* ahrs, float* bx, float* by, float* bz)
{
    *bx = ahrs->integral_fb_x;
    *by = ahrs->integral_fb_y;
    *bz = ahrs->integral_fb_z;
}

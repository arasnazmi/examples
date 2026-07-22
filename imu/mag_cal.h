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

#ifndef __MAG_CAL_H__
#define __MAG_CAL_H__

#include <stdbool.h>
#include <stddef.h>

// Magnetometer calibration.
//
// An uncalibrated magnetometer is useless as a heading reference. Two effects
// dominate, and both are properties of the *board*, not the sensor die:
//
//   * Hard-iron: nearby permanent magnetisation (connectors, shielding cans,
//     traces carrying DC) adds a constant vector to every reading. Rotating the
//     board sweeps out a sphere whose centre is offset from the origin instead
//     of one centred on it. This is the big one -- it maps directly into
//     heading error and can easily reach tens of degrees.
//
//   * Soft-iron: nearby ferrous material distorts the field direction, turning
//     that sphere into an ellipsoid.
//
// Calibration means recovering the sphere: subtract the centre, rescale the
// axes. This module fits the centre with a linear least-squares sphere fit over
// all collected samples, which is far more robust than the widely copied
// min/max approach -- min/max keys the entire result off the two most extreme
// samples on each axis, so a single noise spike corrupts it permanently.

typedef struct
{
    float offset[3]; /*!< Hard-iron offset, subtracted from each reading */
    float scale[3];  /*!< Soft-iron diagonal scale, applied after the offset */
    float radius;    /*!< Fitted field magnitude, uT. Sanity-check value */
} mag_cal_t;

// Collector for the sampling phase. Samples are only accepted once they differ
// enough from the previous one, so holding the board still cannot flood the
// buffer with duplicates and bias the fit toward one orientation.
typedef struct
{
    float* samples;      /*!< Flat xyz triples, 3 * capacity floats */
    size_t capacity;     /*!< Maximum number of samples */
    size_t count;        /*!< Samples accepted so far */
    float min_delta;     /*!< Minimum distance from the previous sample, uT */
    float last[3];       /*!< Previously accepted sample */
    unsigned octant_hit; /*!< Bitmask of the 8 sign-octants covered so far */
    float mid[3];        /*!< Running midpoint used for octant classification */
    float lo[3];         /*!< Running per-axis minimum */
    float hi[3];         /*!< Running per-axis maximum */
} mag_cal_collector_t;

/**
 * @brief Reset a calibration to the identity transform (no correction)
 *
 * @param cal calibration object
 */
void mag_cal_identity(mag_cal_t* cal);

/**
 * @brief Apply a calibration to one raw magnetometer sample
 *
 * Safe to call with `in == out`.
 *
 * @param cal calibration object
 * @param in  raw xyz, uT
 * @param out corrected xyz, uT
 */
void mag_cal_apply(const mag_cal_t* cal, const float in[3], float out[3]);

/**
 * @brief Prepare a collector backed by caller-provided storage
 *
 * @param col       collector object
 * @param storage   buffer of at least 3 * capacity floats, owned by the caller
 * @param capacity  maximum number of samples to retain
 * @param min_delta minimum distance between consecutive accepted samples, uT.
 *                  Around 1.0 works well for the AK09916.
 */
void mag_cal_collector_init(mag_cal_collector_t* col, float* storage, size_t capacity, float min_delta);

/**
 * @brief Offer one raw sample to the collector
 *
 * @param col collector object
 * @param x   raw magnetic field along X, uT
 * @param y   raw magnetic field along Y, uT
 * @param z   raw magnetic field along Z, uT
 *
 * @return true if the sample was accepted, false if rejected as redundant
 */
bool mag_cal_collector_add(mag_cal_collector_t* col, float x, float y, float z);

/**
 * @brief Fraction of the sphere covered so far, as a rough progress indicator
 *
 * Counts how many of the eight sign-octants around the running midpoint have
 * been visited. A fit from samples that only cover part of the sphere is
 * poorly conditioned, so this is what tells the operator when to stop rotating.
 *
 * @param col collector object
 *
 * @return coverage in the range [0, 1]
 */
float mag_cal_collector_coverage(const mag_cal_collector_t* col);

/**
 * @brief Fit a calibration to the collected samples
 *
 * @param col collector object
 * @param cal calibration result, only written on success
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail (too few samples, or a degenerate / ill-conditioned fit)
 */
int mag_cal_solve(const mag_cal_collector_t* col, mag_cal_t* cal);

/**
 * @brief Residual spread of the samples about the fitted sphere
 *
 * Reported as a fraction of the radius. Under roughly 0.05 is a good fit;
 * much above that means the board was moved through a magnetically disturbed
 * area during sampling and the calibration should be redone.
 *
 * @param col collector object
 * @param cal fitted calibration
 *
 * @return normalised RMS residual, or -1.0 if it cannot be computed
 */
float mag_cal_residual(const mag_cal_collector_t* col, const mag_cal_t* cal);

/**
 * @brief Write a calibration to a human-readable text file
 *
 * @param cal  calibration object
 * @param path destination file path
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int mag_cal_save(const mag_cal_t* cal, const char* path);

/**
 * @brief Read a calibration previously written by `mag_cal_save`
 *
 * @param cal  calibration result, only written on success
 * @param path source file path
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail (missing, unreadable, or malformed file)
 */
int mag_cal_load(mag_cal_t* cal, const char* path);

#endif // !__MAG_CAL_H__

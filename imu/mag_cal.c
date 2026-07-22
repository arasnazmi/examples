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

#include "mag_cal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAG_CAL_MIN_SAMPLES 200
#define MAG_CAL_PIVOT_EPSILON 1e-9

void mag_cal_identity(mag_cal_t* cal)
{
    for (int i = 0; i < 3; i++)
    {
        cal->offset[i] = 0.0f;
        cal->scale[i] = 1.0f;
    }
    cal->radius = 0.0f;
}

void mag_cal_apply(const mag_cal_t* cal, const float in[3], float out[3])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = (in[i] - cal->offset[i]) * cal->scale[i];
    }
}

void mag_cal_collector_init(mag_cal_collector_t* col, float* storage, size_t capacity, float min_delta)
{
    memset(col, 0, sizeof(*col));
    col->samples = storage;
    col->capacity = capacity;
    col->min_delta = min_delta;
}

bool mag_cal_collector_add(mag_cal_collector_t* col, float x, float y, float z)
{
    if (col->count >= col->capacity)
    {
        return false;
    }
    if (!isfinite(x) || !isfinite(y) || !isfinite(z))
    {
        return false;
    }

    if (col->count > 0)
    {
        const float dx = x - col->last[0];
        const float dy = y - col->last[1];
        const float dz = z - col->last[2];
        if (dx * dx + dy * dy + dz * dz < col->min_delta * col->min_delta)
        {
            return false;
        }
    }

    col->samples[col->count * 3 + 0] = x;
    col->samples[col->count * 3 + 1] = y;
    col->samples[col->count * 3 + 2] = z;
    col->last[0] = x;
    col->last[1] = y;
    col->last[2] = z;

    const float sample[3] = {x, y, z};
    for (int i = 0; i < 3; i++)
    {
        if (col->count == 0 || sample[i] < col->lo[i])
            col->lo[i] = sample[i];
        if (col->count == 0 || sample[i] > col->hi[i])
            col->hi[i] = sample[i];
        col->mid[i] = 0.5f * (col->lo[i] + col->hi[i]);
    }

    unsigned octant = 0;
    for (int i = 0; i < 3; i++)
    {
        if (sample[i] > col->mid[i])
        {
            octant |= (1u << i);
        }
    }
    col->octant_hit |= (1u << octant);

    col->count++;
    return true;
}

float mag_cal_collector_coverage(const mag_cal_collector_t* col)
{
    int hit = 0;
    for (int i = 0; i < 8; i++)
    {
        if (col->octant_hit & (1u << i))
        {
            hit++;
        }
    }
    return (float)hit / 8.0f;
}

// Solve a small dense system in place by Gaussian elimination with partial
// pivoting. Done in double precision: the normal equations below square the
// sample magnitudes, so float would lose a damaging amount of precision.
static bool mag_cal_solve_linear(double a[4][5], double out[4])
{
    for (int col = 0; col < 4; col++)
    {
        int pivot = col;
        for (int row = col + 1; row < 4; row++)
        {
            if (fabs(a[row][col]) > fabs(a[pivot][col]))
            {
                pivot = row;
            }
        }
        if (fabs(a[pivot][col]) < MAG_CAL_PIVOT_EPSILON)
        {
            return false; // Singular: samples do not constrain the sphere.
        }
        if (pivot != col)
        {
            for (int k = 0; k < 5; k++)
            {
                const double tmp = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }
        for (int row = 0; row < 4; row++)
        {
            if (row == col)
            {
                continue;
            }
            const double factor = a[row][col] / a[col][col];
            for (int k = col; k < 5; k++)
            {
                a[row][k] -= factor * a[col][k];
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        out[i] = a[i][4] / a[i][i];
    }
    return true;
}

int mag_cal_solve(const mag_cal_collector_t* col, mag_cal_t* cal)
{
    if (col->count < MAG_CAL_MIN_SAMPLES)
    {
        return -1;
    }

    // Sphere fit. For a point on a sphere of centre c and radius r:
    //
    //     |m - c|^2 = r^2   ->   x^2+y^2+z^2 = 2*cx*x + 2*cy*y + 2*cz*z + k
    //
    // with k = r^2 - |c|^2. That is linear in the unknowns [2cx, 2cy, 2cz, k],
    // so the least-squares solution comes from the 4x4 normal equations.
    double ata[4][5] = {{0}};
    for (size_t i = 0; i < col->count; i++)
    {
        const double x = col->samples[i * 3 + 0];
        const double y = col->samples[i * 3 + 1];
        const double z = col->samples[i * 3 + 2];
        const double row[4] = {x, y, z, 1.0};
        const double rhs = x * x + y * y + z * z;

        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                ata[r][c] += row[r] * row[c];
            }
            ata[r][4] += row[r] * rhs;
        }
    }

    double solution[4];
    if (!mag_cal_solve_linear(ata, solution))
    {
        return -1;
    }

    const double cx = solution[0] * 0.5;
    const double cy = solution[1] * 0.5;
    const double cz = solution[2] * 0.5;
    const double r_sq = solution[3] + cx * cx + cy * cy + cz * cz;
    if (!(r_sq > 0.0) || !isfinite(r_sq))
    {
        return -1;
    }
    const double radius = sqrt(r_sq);

    // Soft-iron, restricted to a diagonal (per-axis gain). For samples spread
    // evenly over a sphere of radius r, each centred axis has RMS r/sqrt(3);
    // any deviation from that is the ellipsoid's axis ratio.
    double sum_sq[3] = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < col->count; i++)
    {
        const double dx = col->samples[i * 3 + 0] - cx;
        const double dy = col->samples[i * 3 + 1] - cy;
        const double dz = col->samples[i * 3 + 2] - cz;
        sum_sq[0] += dx * dx;
        sum_sq[1] += dy * dy;
        sum_sq[2] += dz * dz;
    }

    double axis_rms[3];
    const double target = radius / sqrt(3.0);
    for (int i = 0; i < 3; i++)
    {
        axis_rms[i] = sqrt(sum_sq[i] / (double)col->count);
        if (!(axis_rms[i] > 0.0) || !isfinite(axis_rms[i]))
        {
            return -1;
        }
    }

    // Normalise the gains so their geometric mean is 1, which keeps the overall
    // field magnitude intact and leaves only the axis ratios corrected.
    const double geo_mean = cbrt((target / axis_rms[0]) * (target / axis_rms[1]) * (target / axis_rms[2]));
    if (!(geo_mean > 0.0) || !isfinite(geo_mean))
    {
        return -1;
    }

    cal->offset[0] = (float)cx;
    cal->offset[1] = (float)cy;
    cal->offset[2] = (float)cz;
    for (int i = 0; i < 3; i++)
    {
        cal->scale[i] = (float)((target / axis_rms[i]) / geo_mean);
    }
    cal->radius = (float)radius;

    return 0;
}

float mag_cal_residual(const mag_cal_collector_t* col, const mag_cal_t* cal)
{
    if (col->count == 0 || !(cal->radius > 0.0f))
    {
        return -1.0f;
    }

    double sum_sq = 0.0;
    for (size_t i = 0; i < col->count; i++)
    {
        const float in[3] = {col->samples[i * 3 + 0], col->samples[i * 3 + 1], col->samples[i * 3 + 2]};
        float out[3];
        mag_cal_apply(cal, in, out);

        const double norm = sqrt((double)out[0] * out[0] + (double)out[1] * out[1] + (double)out[2] * out[2]);
        const double err = norm - (double)cal->radius;
        sum_sq += err * err;
    }

    return (float)(sqrt(sum_sq / (double)col->count) / (double)cal->radius);
}

int mag_cal_save(const mag_cal_t* cal, const char* path)
{
    FILE* file = fopen(path, "w");
    if (file == NULL)
    {
        return -1;
    }

    fprintf(file, "# ICM-20948 / AK09916 magnetometer calibration\n");
    fprintf(file, "# corrected = (raw - offset) * scale, values in uT\n");
    fprintf(file, "offset %.6f %.6f %.6f\n", (double)cal->offset[0], (double)cal->offset[1], (double)cal->offset[2]);
    fprintf(file, "scale %.6f %.6f %.6f\n", (double)cal->scale[0], (double)cal->scale[1], (double)cal->scale[2]);
    fprintf(file, "radius %.6f\n", (double)cal->radius);

    if (fclose(file) != 0)
    {
        return -1;
    }
    return 0;
}

int mag_cal_load(mag_cal_t* cal, const char* path)
{
    FILE* file = fopen(path, "r");
    if (file == NULL)
    {
        return -1;
    }

    mag_cal_t parsed;
    mag_cal_identity(&parsed);
    bool have_offset = false;
    bool have_scale = false;
    char line[256];

    while (fgets(line, sizeof(line), file) != NULL)
    {
        if (line[0] == '#' || line[0] == '\n')
        {
            continue;
        }

        double v0, v1, v2;
        if (sscanf(line, "offset %lf %lf %lf", &v0, &v1, &v2) == 3)
        {
            parsed.offset[0] = (float)v0;
            parsed.offset[1] = (float)v1;
            parsed.offset[2] = (float)v2;
            have_offset = true;
        }
        else if (sscanf(line, "scale %lf %lf %lf", &v0, &v1, &v2) == 3)
        {
            parsed.scale[0] = (float)v0;
            parsed.scale[1] = (float)v1;
            parsed.scale[2] = (float)v2;
            have_scale = true;
        }
        else if (sscanf(line, "radius %lf", &v0) == 1)
        {
            parsed.radius = (float)v0;
        }
    }

    fclose(file);

    if (!have_offset || !have_scale)
    {
        return -1;
    }
    for (int i = 0; i < 3; i++)
    {
        if (!isfinite(parsed.offset[i]) || !isfinite(parsed.scale[i]) || !(parsed.scale[i] > 0.0f))
        {
            return -1;
        }
    }

    *cal = parsed;
    return 0;
}

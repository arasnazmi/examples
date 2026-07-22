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

#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "stdint.h"
#include <stdbool.h>
#define ICM20948_WHO_AM_I_VAL 0xEA
#define ICM20948_MAG_ADDRESS 0x0C
#define ICM20948_MAG_WIA_1_VAL 0x48 /*!< AK09916 company ID */
#define ICM20948_MAG_WIA_2_VAL 0x09 /*!< AK09916 device ID */

/* Registers ICM20948 USER BANK 0*/
#define ICM20948_WHO_AM_I 0x00
#define ICM20948_USER_CTRL 0x03
#define ICM20948_LP_CONFIG 0x05
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_INT_PIN_CFG 0x0F
#define ICM20948_I2C_MST_STATUS 0x17
#define ICM20948_ACCEL_OUT 0x2D // accel data registers begin
#define ICM20948_GYRO_OUT 0x33  // gyro data registers begin

/* Registers ICM20948 USER BANK 2*/
#define ICM20948_GYRO_SMPLRT_DIV 0x00
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_ODR_ALIGN_EN 0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG 0x14

/* Registers ICM20948 USER BANK 3*/
#define ICM20948_I2C_MST_ODR_CFG 0x00
#define ICM20948_I2C_MST_CTRL 0x01
#define ICM20948_I2C_SLV0_ADDR 0x03
#define ICM20948_I2C_SLV0_REG 0x04
#define ICM20948_I2C_SLV0_CTRL 0x05
#define ICM20948_I2C_SLV4_ADDR 0x13
#define ICM20948_I2C_SLV4_REG 0x14
#define ICM20948_I2C_SLV4_CTRL 0x15
#define ICM20948_I2C_SLV4_DO 0x16
#define ICM20948_I2C_SLV4_DI 0x17

/* Registers AK09916 */
#define ICM20948_MAG_WIA_1 0x00 // Who I am, Company ID
#define ICM20948_MAG_WIA_2 0x01 // Who I am, Device ID
#define ICM20948_MAG_STATUS_1 0x10
#define ICM20948_MAG_CNTL_2 0x31
#define ICM20948_MAG_CNTL_3 0x32

/* Register bits, grouped by the register they belong to */

/* PWR_MGMT_1 */
#define ICM20948_RESET 0x80
#define ICM20948_SLEEP 0x40
#define ICM20948_LP_EN 0x20 /*!< Low power mode; required for duty cycled operation */

/* USER_CTRL */
#define ICM20948_I2C_MST_EN 0x20  /*!< Enable the auxiliary I2C master */
#define ICM20948_I2C_IF_DIS 0x10  /*!< Disable the primary I2C slave interface (SPI only) */
#define ICM20948_I2C_MST_RST 0x02 /*!< Reset the auxiliary I2C master, self clearing */

/* INT_PIN_CFG */
#define ICM20948_BYPASS_EN 0x02 /*!< Tie the auxiliary bus to the host bus; blocks the master */

/* LP_CONFIG */
#define ICM20948_I2C_MST_CYCLE 0x40 /*!< Run the auxiliary master off its own ODR */

/* I2C_SLVx_CTRL / I2C_SLV4_CTRL */
#define ICM20948_I2C_SLVX_EN 0x80 /*!< Arm the channel */

/* I2C_MST_STATUS */
#define ICM20948_I2C_SLV4_DONE 0x40 /*!< SLV4 transaction completed */
#define ICM20948_I2C_SLV4_NACK 0x10 /*!< SLV4 transaction was not acknowledged */

/* AK09916 ST1 / ST2 */
#define ICM20948_MAG_DRDY 0x01 /*!< ST1: a new measurement is ready */
#define ICM20948_MAG_OVF 0x08  /*!< ST2: the sensor overflowed, sample is invalid */

/* Auxiliary I2C address modifier */
#define ICM20948_MAG_READ 0x80 /*!< OR into a slave address to make the transaction a read */

/* Registers ICM20948 ALL BANKS */
#define ICM20948_REG_BANK_SEL 0x7F

/*!< AK09916 sensitivity, microtesla per LSB. Fixed; the part has no
     sensitivity-adjustment ROM (unlike the older AK8963). */
#define ICM20948_MAG_SENSITIVITY 0.15f

/*!< Number of EXT_SLV_SENS_DATA bytes the magnetometer occupies:
     ST1, HXL..HZH, TMPS, ST2. */
#define ICM20948_MAG_DATA_LEN 9

/*!< Base rate for the accel/gyro output data rate divider, Hz. */
#define ICM20948_ODR_BASE_HZ 1125.0f

typedef enum
{
    ACCE_FS_2G = 0,  /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G = 1,  /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G = 2,  /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
} icm20948_acce_fs_t;

typedef enum
{
    GYRO_FS_250DPS = 0,  /*!< Gyroscope full scale range is +/- 250 degree per second */
    GYRO_FS_500DPS = 1,  /*!< Gyroscope full scale range is +/- 500 degree per second */
    GYRO_FS_1000DPS = 2, /*!< Gyroscope full scale range is +/- 1000 degree per second */
    GYRO_FS_2000DPS = 3, /*!< Gyroscope full scale range is +/- 2000 degree per second */
} icm20948_gyro_fs_t;

typedef enum
{
    ICM20948_DLPF_0,
    ICM20948_DLPF_1,
    ICM20948_DLPF_2,
    ICM20948_DLPF_3,
    ICM20948_DLPF_4,
    ICM20948_DLPF_5,
    ICM20948_DLPF_6,
    ICM20948_DLPF_7,
    ICM20948_DLPF_OFF
} icm20948_dlpf_t;

typedef enum
{
    ICM20948_MAG_PWR_DOWN = 0x00,
    ICM20948_MAG_TRIGGER_MODE = 0x01,
    ICM20948_MAG_CONT_MODE_10HZ = 0x02,
    ICM20948_MAG_CONT_MODE_20HZ = 0x04,
    ICM20948_MAG_CONT_MODE_50HZ = 0x06,
    ICM20948_MAG_CONT_MODE_100HZ = 0x08
} icm20948_mag_mode_t;

typedef struct
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t mx_raw; /*!< Magnetometer counts, already mapped into the accel/gyro frame */
    int16_t my_raw;
    int16_t mz_raw;
    float ax; /*!< Acceleration, g */
    float ay;
    float az;
    float gx; /*!< Angular rate, degrees/second, bias-corrected */
    float gy;
    float gz;
    float mx; /*!< Magnetic field, microtesla, in the accel/gyro frame */
    float my;
    float mz;
    float gx_bias; /*!< Gyro zero-rate offset, degrees/second, from icm20948_calibrate_gyro */
    float gy_bias;
    float gz_bias;
    float temp;      /*!< Die temperature, degrees Celsius */
    bool mag_valid;  /*!< Last magnetometer sample was fresh and not overflowed */
    bool gyro_calibrated;
} icm20948_data_t;

typedef void* icm20948_handle_t;

typedef struct
{
    icm20948_acce_fs_t acce_fs;   /*!< Accelerometer full scale range */
    icm20948_gyro_fs_t gyro_fs;   /*!< Gyroscope full scale range */
    icm20948_dlpf_t acce_dlpf;    /*!< Accelerometer low pass filter, ICM20948_DLPF_OFF to bypass */
    icm20948_dlpf_t gyro_dlpf;    /*!< Gyroscope low pass filter, ICM20948_DLPF_OFF to bypass */
    uint8_t sample_rate_div;      /*!< Output data rate = 1125 / (1 + sample_rate_div) Hz */
    bool enable_mag;              /*!< Bring up the AK09916 on the auxiliary I2C bus */
    icm20948_mag_mode_t mag_mode; /*!< Magnetometer measurement mode when enable_mag is set */
    bool mag_required;            /*!< Treat a magnetometer bring-up failure as fatal */
    bool mag_debug;               /*!< On failure, probe every auxiliary clocking arrangement */
} icm20948_config_t;

typedef struct
{
    int fd;
    char* tag;
    int bank;
    bool mag_enabled;
    bool temp_valid;
    uint8_t aux_status; /*!< Last I2C_MST_STATUS seen by an auxiliary transaction, for diagnostics */
    icm20948_acce_fs_t acce_fs;
    icm20948_gyro_fs_t gyro_fs;
    icm20948_data_t* data;
} icm20948_dev_t;

/**
 * @brief Fill a configuration with defaults suited to orientation tracking
 *
 * 250 dps / 2 g full scale (the lowest ranges, so the least quantisation noise
 * for a device that is not being thrown around), 51 Hz low pass filters, and a
 * 102 Hz output data rate matched to the magnetometer's 100 Hz.
 *
 * @param config configuration to populate
 */
void icm20948_config_default(icm20948_config_t* config);

/**
 * @brief Initialize the SPI bus and device
 *
 * @param sensor object handle of icm20948
 * @param dev_path SPI device path
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_spi_bus_init(icm20948_handle_t sensor, const char* dev_path);

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param data sensor data structure
 * @param tag sensor tag for logging
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
icm20948_handle_t icm20948_create(icm20948_data_t* data, char* tag);

/**
 * @brief Reset and bring the sensor up according to the given configuration
 *
 * Performs the full bring-up: reset, wake, identity check, full scale ranges,
 * low pass filters, output data rate, and -- if requested -- the auxiliary I2C
 * master and the AK09916 magnetometer. Safe to call again to recover a device
 * that has fallen off the bus.
 *
 * @param icm20948 object handle of icm20948
 * @param config configuration, or NULL for `icm20948_config_default`
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_configure(icm20948_handle_t icm20948, const icm20948_config_t* config);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of icm20948
 */
void icm20948_delete(icm20948_handle_t sensor);

/**
 * @brief Get device identification of icm20948
 *
 * @param sensor object handle of icm20948
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t* deviceid);

/**
 * @brief Wake up icm20948
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_wake_up(icm20948_handle_t sensor);

/**
 * @brief Set gyroscope full scale range
 *
 * @param sensor object handle of icm20948
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of icm20948
 *
 * @return gyroscope sensitivity
 */
float icm20948_get_gyro_sensitivity(icm20948_handle_t sensor);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_get_gyro(icm20948_handle_t sensor);

/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of icm20948
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t acce_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of icm20948
 *
 * @return accelerometer sensitivity
 */
float icm20948_get_acce_sensitivity(icm20948_handle_t sensor);

/**
 * @brief Read accelerometer, gyroscope, magnetometer and temperature at once
 *
 * All of these live in one contiguous register block, so a single bus
 * transaction fetches the lot. Besides being much faster than four separate
 * reads, it guarantees the samples describe the same instant -- which matters,
 * because feeding an orientation filter accel and mag vectors captured
 * milliseconds apart injects error during motion.
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_get_agmt(icm20948_handle_t sensor);

/**
 * @brief Bring up the auxiliary I2C master and the AK09916 magnetometer
 *
 * Called automatically by `icm20948_configure` when `enable_mag` is set.
 * Afterwards the magnetometer streams into the EXT_SLV_SENS_DATA registers with
 * no further bus traffic, so `icm20948_get_agmt` picks it up for free.
 *
 * @param sensor object handle of icm20948
 * @param mode measurement mode, typically ICM20948_MAG_CONT_MODE_100HZ
 * @param debug probe every auxiliary clocking arrangement if bring-up fails
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_mag_init(icm20948_handle_t sensor, icm20948_mag_mode_t mode, bool debug);

/**
 * @brief Report whether the magnetometer was brought up successfully
 *
 * False either because it was not requested or because bring-up failed. When
 * false the magnetometer fields in icm20948_data_t stay zero and yaw has no
 * absolute reference.
 *
 * @param sensor object handle of icm20948
 *
 * @return true if magnetometer samples are being streamed
 */
bool icm20948_mag_available(icm20948_handle_t sensor);

/**
 * @brief Measure and store the gyroscope zero-rate offset
 *
 * The board must be completely still. Every gyroscope reads a small non-zero
 * rate at rest, and integrating that offset is what turns into angle drift, so
 * removing it is the single biggest improvement available to a gyro signal.
 * Fails rather than storing a bad offset if motion is detected.
 *
 * The offset is temperature dependent, so it is measured at startup rather than
 * stored to disk. Slow residual drift after this is handled by the AHRS
 * filter's integral term.
 *
 * @param sensor object handle of icm20948
 * @param samples number of samples to average, 1000 is a good default
 * @param interval_us delay between samples, matched to the output data rate
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail (bus error, or the board moved during the measurement)
 */
int icm20948_calibrate_gyro(icm20948_handle_t sensor, int samples, int interval_us);

/**
 * @brief Reset the internal registers and restores the default settings
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_reset(icm20948_handle_t sensor);

/**
 * @brief Select USER BANK.
 * 0: Select USER BANK 0.
 * 1: Select USER BANK 1.
 * 2: Select USER BANK 2.
 * 3: Select USER BANK 3.
 *
 * @param sensor object handle of icm20948
 * @param bank   user bank number
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_bank(icm20948_handle_t sensor, uint8_t bank);

/**
 * @brief Set the accelerometer and gyroscope output data rate divider
 *
 * Output data rate = 1125 / (1 + div) Hz for both sensors.
 *
 * @param sensor object handle of icm20948
 * @param div    rate divider
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_sample_rate_div(icm20948_handle_t sensor, uint8_t div);

/**
 * @brief Configure low pass filter for accelerometer
 *
 * @param sensor      object handle of icm20948
 * @param dlpf_acce   dlpf configuration for accelerometer
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_acce_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_acce);

/**
 * @brief Configure low pass filter for gyroscope
 *
 * @param sensor      object handle of icm20948
 * @param dlpf_gyro   dlpf configuration for gyroscope
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_set_gyro_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_gyro);

/**
 * @brief Check if the sensor is online
 *
 * @param sensor object handle of icm20948
 *
 * @return
 *     - 0 Success
 *     - not 0 Fail
 */
int icm20948_check_online(icm20948_handle_t sensor);

#endif // !__ICM20948_H__

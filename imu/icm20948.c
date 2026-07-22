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

#include "icm20948.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/*!< Largest single bus read this driver issues, in bytes. Sized for the
     accel + gyro + temp + magnetometer burst in icm20948_get_agmt. */
#define ICM20948_MAX_TRANSFER 32

/*!< Time for the auxiliary I2C master to complete one queued transaction. */
#define ICM20948_MAG_XFER_WAIT_US 10000

/*!< The AK09916 needs ~100 us to come out of a soft reset; be generous. */
#define ICM20948_MAG_RESET_WAIT_US 100000

/*!< Poll interval and cap while waiting on an auxiliary I2C transaction.
     250 * 200 us = 50 ms, far longer than a transaction can legitimately take
     but short enough that a dead bus is reported promptly. */
#define ICM20948_AUX_TXN_POLL_US 200
#define ICM20948_AUX_TXN_MAX_POLLS 250

/*!< Magnetometer identification attempts, with a master reset between each.
     Bring-up is genuinely flaky when a previous run left a channel armed. */
#define ICM20948_MAG_START_ATTEMPTS 5

/*!< Register span from ACCEL_OUT through the last magnetometer data byte. */
#define ICM20948_AGT_LEN 14
#define ICM20948_AGMT_LEN (ICM20948_AGT_LEN + ICM20948_MAG_DATA_LEN)

/*!< Peak-to-peak gyro spread tolerated during bias calibration, deg/s. */
#define ICM20948_GYRO_CAL_MAX_SPREAD 1.5f

static const uint32_t spi_speed = 10000000;
static const uint8_t spi_bits = 8;
static const uint16_t spi_delay = 0;
static const uint8_t spi_mode = SPI_MODE_3;

static int icm20948_write_spi(icm20948_handle_t sensor, const uint8_t reg_start_addr, const uint8_t data_buf)
{
    int ret;
    uint8_t write_buf[2];
    write_buf[0] = reg_start_addr;
    write_buf[1] = data_buf;
    const icm20948_dev_t* sens = sensor;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)write_buf,
        .rx_buf = 0,
        .len = 2,
        .speed_hz = spi_speed,
        .delay_usecs = spi_delay,
        .bits_per_word = spi_bits,
        .cs_change = 0,
    };

    ret = ioctl(sens->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
    {
        fprintf(stderr, "%s Failed to write to the SPI bus.\n", sens->tag);
        return -1;
    }

    return 0;
}

static int icm20948_read_spi(icm20948_handle_t sensor, const uint8_t reg_start_addr, uint8_t* data_buf,
                             const uint8_t data_len)
{
    int ret;
    // Both buffers must cover the whole transfer: the controller clocks out
    // data_len + 1 bytes regardless of how few are meaningful, and it reads
    // every one of them from tx_buf.
    uint8_t tx_buf[ICM20948_MAX_TRANSFER + 1] = {0};
    uint8_t rx_buf[ICM20948_MAX_TRANSFER + 1] = {0};
    const icm20948_dev_t* sens = sensor;

    if (data_len > ICM20948_MAX_TRANSFER)
    {
        fprintf(stderr, "%s SPI read of %u bytes exceeds the %u byte limit.\n", sens->tag, data_len,
                ICM20948_MAX_TRANSFER);
        return -1;
    }

    tx_buf[0] = reg_start_addr | 0x80;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = data_len + 1u,
        .speed_hz = spi_speed,
        .delay_usecs = spi_delay,
        .bits_per_word = spi_bits,
        .cs_change = 0,
    };

    ret = ioctl(sens->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0)
    {
        fprintf(stderr, "%s Failed to read from the SPI bus.\n", sens->tag);
        return -1;
    }
    memcpy(data_buf, rx_buf + 1, data_len);
    return 0;
}

int icm20948_spi_bus_init(icm20948_handle_t sensor, const char* dev_path)
{
    icm20948_dev_t* sens = sensor;
    int ret;
    sens->fd = open(dev_path, O_RDWR);
    if (sens->fd < 0)
    {
        fprintf(stderr, "%s Failed to open SPI device\n", sens->tag);
        return -1;
    }
    ret = ioctl(sens->fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret < 0)
    {
        fprintf(stderr, "%s Failed to set SPI mode\n", sens->tag);
        close(sens->fd);
        return -1;
    }
    ret = ioctl(sens->fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    if (ret < 0)
    {
        fprintf(stderr, "%s Failed to set SPI bits per word\n", sens->tag);
        close(sens->fd);
        return -1;
    }
    ret = ioctl(sens->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (ret < 0)
    {
        fprintf(stderr, "%s Failed to set SPI speed\n", sens->tag);
        close(sens->fd);
        return -1;
    }
    fprintf(stdout, "%s SPI bus and device initialized successfully\n", sens->tag);
    return ret;
}

icm20948_handle_t icm20948_create(icm20948_data_t* data, char* tag)
{
    icm20948_dev_t* sensor = (icm20948_dev_t*)calloc(1, sizeof(icm20948_dev_t));
    if (!sensor)
    {
        fprintf(stderr, "%s Memory allocation failed\n", tag);
        return NULL;
    }

    sensor->tag = tag;
    sensor->data = data;
    sensor->bank = -1;
    sensor->fd = -1; // Not 0: that is stdin, which icm20948_delete would close.
    memset(data, 0, sizeof(*data));

    return (icm20948_handle_t)sensor;
}

void icm20948_delete(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = sensor;
    if (sens == NULL)
    {
        return;
    }
    if (sens->fd >= 0)
    {
        close(sens->fd);
    }
    free(sens);
}

void icm20948_config_default(icm20948_config_t* config)
{
    // The lowest full scale ranges are deliberate. At +/-2000 dps one gyro LSB
    // is 0.061 deg/s; at +/-250 dps it is 0.0076 deg/s. Since the whole point
    // here is integrating small rotations accurately, the narrow range wins
    // unless the application genuinely spins faster than 250 deg/s.
    config->acce_fs = ACCE_FS_2G;
    config->gyro_fs = GYRO_FS_250DPS;

    // 51 Hz bandwidth against a 102 Hz sample rate: comfortably inside Nyquist,
    // so vibration above the sample rate is attenuated instead of aliasing down
    // into the signal band as a false slow rotation.
    config->acce_dlpf = ICM20948_DLPF_3;
    config->gyro_dlpf = ICM20948_DLPF_3;

    config->sample_rate_div = 10; // 1125 / 11 = 102.3 Hz
    config->enable_mag = true;
    config->mag_mode = ICM20948_MAG_CONT_MODE_100HZ;

    // A magnetometer failure degrades yaw to dead reckoning but leaves the
    // accelerometer and gyroscope perfectly usable, so it is a warning by
    // default rather than a refusal to start.
    config->mag_required = false;
    config->mag_debug = false;
}

int icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t* const deviceid)
{
    icm20948_dev_t* sens = sensor;
    if (sens->bank != 0)
    {
        icm20948_set_bank(sensor, 0);
    }
    return icm20948_read_spi(sensor, ICM20948_WHO_AM_I, deviceid, 1);
}

int icm20948_wake_up(icm20948_handle_t sensor)
{
    int ret;
    uint8_t tmp;
    ret = icm20948_read_spi(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
    if (0 != ret)
    {
        return ret;
    }
    tmp &= (uint8_t)(~ICM20948_SLEEP);
    ret = icm20948_write_spi(sensor, ICM20948_PWR_MGMT_1, tmp);
    return ret;
}

int icm20948_reset(icm20948_handle_t sensor)
{
    int ret;
    uint8_t tmp;
    icm20948_dev_t* sens = sensor;

    ret = icm20948_read_spi(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
    if (ret != 0)
        return ret;
    tmp |= ICM20948_RESET;
    ret = icm20948_write_spi(sensor, ICM20948_PWR_MGMT_1, tmp);
    if (ret != 0)
        return ret;

    // The reset clears the bank selection too, so the cached value is stale.
    sens->bank = -1;
    sens->mag_enabled = false;
    return ret;
}

int icm20948_set_bank(icm20948_handle_t sensor, uint8_t bank)
{
    int ret;
    icm20948_dev_t* sens = sensor;
    if (bank > 3)
        return -1;
    ret = icm20948_write_spi(sensor, ICM20948_REG_BANK_SEL, (uint8_t)((bank << 4) & 0x30));
    if (ret != 0)
        return ret;
    // Cache the logical bank number, not the shifted register value, so that
    // comparisons like `bank != 0` elsewhere mean what they appear to mean.
    sens->bank = (int)bank;
    return ret;
}

int icm20948_set_sample_rate_div(icm20948_handle_t sensor, uint8_t div)
{
    int ret;

    ret = icm20948_set_bank(sensor, 2);
    if (ret != 0)
        return ret;

    ret = icm20948_write_spi(sensor, ICM20948_GYRO_SMPLRT_DIV, div);
    if (ret != 0)
        return ret;

    // The accelerometer divider is 12 bits split across two registers.
    ret = icm20948_write_spi(sensor, ICM20948_ACCEL_SMPLRT_DIV_1, 0);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_ACCEL_SMPLRT_DIV_2, div);
    if (ret != 0)
        return ret;

    // Start both sensors' sample clocks together so their samples stay in step.
    ret = icm20948_write_spi(sensor, ICM20948_ODR_ALIGN_EN, 0x01);
    if (ret != 0)
        return ret;

    return icm20948_set_bank(sensor, 0);
}

/* ------------------------------------------------------------------------- */
/* Auxiliary I2C bus / AK09916 magnetometer                                   */
/*                                                                            */
/* The magnetometer is a physically separate die with its own I2C address. It  */
/* is not reachable over the host bus at all -- the only path to it is the     */
/* ICM-20948's built-in I2C master, which acts as a proxy.                     */
/*                                                                            */
/* Two channels are used, for two different jobs:                             */
/*                                                                            */
/*   SLV4 for configuration. It is the only channel with completion and NACK   */
/*   status bits, so a transaction can be waited on and its failure detected   */
/*   rather than guessed at with a fixed delay. Auxiliary transactions only    */
/*   run on a master cycle, so the time they take depends on the configured    */
/*   ODR -- a blind sleep is a race.                                          */
/*                                                                            */
/*   SLV0 for streaming. Once armed it is left running, and the master copies  */
/*   the magnetometer's output registers into EXT_SLV_SENS_DATA_* every cycle  */
/*   with no host involvement.                                                */
/* ------------------------------------------------------------------------- */

// Reset the auxiliary I2C master state machine. Used to recover the bus when a
// previous run, or a failed transaction, left a channel armed mid-transfer.
static int icm20948_aux_master_reset(icm20948_handle_t sensor)
{
    int ret;
    uint8_t tmp;

    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;
    ret = icm20948_read_spi(sensor, ICM20948_USER_CTRL, &tmp, 1);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_USER_CTRL, (uint8_t)(tmp | ICM20948_I2C_MST_RST));
    if (ret != 0)
        return ret;
    usleep(ICM20948_MAG_XFER_WAIT_US);
    return 0;
}

// Connect or disconnect the auxiliary bus pins from the host bus. While
// pass-through is enabled the internal master cannot drive the auxiliary bus,
// so every transaction silently does nothing.
static int icm20948_set_bypass(icm20948_handle_t sensor, bool enable)
{
    int ret;
    uint8_t tmp;

    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;
    ret = icm20948_read_spi(sensor, ICM20948_INT_PIN_CFG, &tmp, 1);
    if (ret != 0)
        return ret;
    if (enable)
        tmp |= ICM20948_BYPASS_EN;
    else
        tmp &= (uint8_t)~ICM20948_BYPASS_EN;
    return icm20948_write_spi(sensor, ICM20948_INT_PIN_CFG, tmp);
}

/**
 * @brief Perform a single byte transaction on the auxiliary I2C bus via SLV4.
 *
 * Waits for the master to report completion and distinguishes a device that
 * did not acknowledge from a master that never ran at all.
 *
 * @return 0 on success, -1 on NACK or timeout, negative errno on bus error.
 */
static int icm20948_aux_txn(icm20948_handle_t sensor, uint8_t addr, uint8_t reg, uint8_t* value, bool read)
{
    int ret;
    unsigned polls;
    uint8_t status = 0;
    icm20948_dev_t* sens = sensor;

    // I2C_MST_STATUS clears on read. Drain it first, so that a completion bit
    // left over from the previous transaction is not mistaken for this one
    // finishing before it has even started.
    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;
    ret = icm20948_read_spi(sensor, ICM20948_I2C_MST_STATUS, &status, 1);
    if (ret != 0)
        return ret;
    status = 0;

    ret = icm20948_set_bank(sensor, 3);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV4_ADDR,
                               (uint8_t)(read ? (addr | ICM20948_MAG_READ) : addr));
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV4_REG, reg);
    if (ret != 0)
        return ret;
    if (!read)
    {
        ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV4_DO, *value);
        if (ret != 0)
            return ret;
    }
    // No delay, no interrupt, register address sent: just enable and wait.
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN);
    if (ret != 0)
        return ret;

    for (polls = 0; polls < ICM20948_AUX_TXN_MAX_POLLS; polls++)
    {
        ret = icm20948_set_bank(sensor, 0);
        if (ret != 0)
            return ret;
        // I2C_MST_STATUS clears on read, so this one read has to capture both
        // the completion bit and the NACK bit.
        ret = icm20948_read_spi(sensor, ICM20948_I2C_MST_STATUS, &status, 1);
        if (ret != 0)
            return ret;
        if ((status & ICM20948_I2C_SLV4_DONE) != 0)
            break;
        usleep(ICM20948_AUX_TXN_POLL_US);
    }
    sens->aux_status = status;

    if ((status & ICM20948_I2C_SLV4_DONE) == 0)
    {
        // Disarm, otherwise the stalled transaction is retried every cycle.
        if (icm20948_set_bank(sensor, 3) == 0)
            (void)icm20948_write_spi(sensor, ICM20948_I2C_SLV4_CTRL, 0);
        (void)icm20948_set_bank(sensor, 0);
        return -1;
    }
    if ((status & ICM20948_I2C_SLV4_NACK) != 0)
        return -1;

    if (read)
    {
        ret = icm20948_set_bank(sensor, 3);
        if (ret != 0)
            return ret;
        ret = icm20948_read_spi(sensor, ICM20948_I2C_SLV4_DI, value, 1);
        if (ret != 0)
            return ret;
    }

    return icm20948_set_bank(sensor, 0);
}

static int icm20948_mag_write(icm20948_handle_t sensor, uint8_t reg, uint8_t value)
{
    return icm20948_aux_txn(sensor, ICM20948_MAG_ADDRESS, reg, &value, false);
}

static int icm20948_mag_read(icm20948_handle_t sensor, uint8_t reg, uint8_t* value)
{
    return icm20948_aux_txn(sensor, ICM20948_MAG_ADDRESS, reg, value, true);
}

// Confirm that bank 3 register writes actually land. If they do not, nothing
// about the auxiliary master can be trusted, because every one of its control
// registers lives in that bank.
static bool icm20948_aux_bank_writable(icm20948_handle_t sensor)
{
    uint8_t readback = 0;
    const uint8_t pattern = 0x5A;

    if (icm20948_set_bank(sensor, 3) != 0)
        return false;
    if (icm20948_write_spi(sensor, ICM20948_I2C_SLV4_REG, pattern) != 0)
        return false;
    if (icm20948_read_spi(sensor, ICM20948_I2C_SLV4_REG, &readback, 1) != 0)
        return false;
    (void)icm20948_set_bank(sensor, 0);

    return readback == pattern;
}

/**
 * @brief Try one auxiliary master clocking arrangement and report the outcome.
 *
 * The master only runs transactions on a cycle, and which clock drives that
 * cycle depends on LP_CONFIG and PWR_MGMT_1 together. Rather than guess, try
 * each documented combination and report which one -- if any -- actually
 * completes a transaction.
 */
static void icm20948_aux_probe(icm20948_dev_t* sens, const char* label, uint8_t lp_config, uint8_t mst_odr,
                               bool low_power)
{
    icm20948_handle_t sensor = sens;
    uint8_t pwr = 0;
    uint8_t wia = 0;
    int ret;

    if (icm20948_set_bank(sensor, 0) != 0)
        return;
    if (icm20948_read_spi(sensor, ICM20948_PWR_MGMT_1, &pwr, 1) != 0)
        return;
    if (low_power)
        pwr |= ICM20948_LP_EN;
    else
        pwr &= (uint8_t)~ICM20948_LP_EN;
    (void)icm20948_write_spi(sensor, ICM20948_PWR_MGMT_1, pwr);
    (void)icm20948_write_spi(sensor, ICM20948_LP_CONFIG, lp_config);

    if (icm20948_set_bank(sensor, 3) == 0)
        (void)icm20948_write_spi(sensor, ICM20948_I2C_MST_ODR_CFG, mst_odr);
    (void)icm20948_set_bank(sensor, 0);
    usleep(ICM20948_MAG_XFER_WAIT_US);

    ret = icm20948_mag_read(sensor, ICM20948_MAG_WIA_1, &wia);

    fprintf(stderr, "%s     %-34s MST_STATUS=0x%02X WIA1=0x%02X %s\n", sens->tag, label, sens->aux_status, wia,
            (ret == 0 && wia == ICM20948_MAG_WIA_1_VAL) ? "<-- WORKS" : "");
}

// Dump the registers that decide whether the auxiliary master can talk at all,
// then work through the clocking arrangements that could explain a master that
// never runs. Printed only when bring-up fails, so that a failure identifies
// its cause instead of just reporting "no id".
static void icm20948_mag_report_failure(icm20948_dev_t* sens, bool verbose)
{
    uint8_t user_ctrl = 0;
    uint8_t int_pin_cfg = 0;
    uint8_t lp_config = 0;
    uint8_t pwr_mgmt_1 = 0;
    uint8_t mst_ctrl = 0;
    icm20948_handle_t sensor = sens;

    if (icm20948_set_bank(sensor, 0) == 0)
    {
        (void)icm20948_read_spi(sensor, ICM20948_USER_CTRL, &user_ctrl, 1);
        (void)icm20948_read_spi(sensor, ICM20948_INT_PIN_CFG, &int_pin_cfg, 1);
        (void)icm20948_read_spi(sensor, ICM20948_LP_CONFIG, &lp_config, 1);
        (void)icm20948_read_spi(sensor, ICM20948_PWR_MGMT_1, &pwr_mgmt_1, 1);
    }
    if (icm20948_set_bank(sensor, 3) == 0)
        (void)icm20948_read_spi(sensor, ICM20948_I2C_MST_CTRL, &mst_ctrl, 1);
    (void)icm20948_set_bank(sensor, 0);

    fprintf(stderr, "%s   USER_CTRL=0x%02X INT_PIN_CFG=0x%02X LP_CONFIG=0x%02X PWR_MGMT_1=0x%02X\n", sens->tag,
            user_ctrl, int_pin_cfg, lp_config, pwr_mgmt_1);
    fprintf(stderr, "%s   I2C_MST_CTRL=0x%02X (bank 3) I2C_MST_STATUS=0x%02X\n", sens->tag, mst_ctrl, sens->aux_status);

    if ((sens->aux_status & ICM20948_I2C_SLV4_DONE) != 0 && (sens->aux_status & ICM20948_I2C_SLV4_NACK) != 0)
    {
        fprintf(stderr,
                "%s   The auxiliary bus is running but nothing answered at 0x%02X. The\n"
                "%s   magnetometer is unpowered, absent, or not on the auxiliary bus.\n",
                sens->tag, ICM20948_MAG_ADDRESS, sens->tag);
        return;
    }

    if (!verbose)
    {
        fprintf(stderr,
                "%s   The auxiliary I2C master never completed a transaction, so nothing\n"
                "%s   reached the magnetometer. Re-run with --mag-debug to probe further.\n",
                sens->tag, sens->tag);
        return;
    }

    // Master never completed a transaction. Establish whether its control
    // registers are even reachable before blaming the clocking.
    if (!icm20948_aux_bank_writable(sensor))
    {
        fprintf(stderr,
                "%s   Bank 3 register writes are not taking effect, so the auxiliary\n"
                "%s   master cannot be configured at all. This is a host bus problem,\n"
                "%s   not a magnetometer problem.\n",
                sens->tag, sens->tag, sens->tag);
        return;
    }

    fprintf(stderr,
            "%s   Bank 3 is writable but the master never cycles. Trying each\n"
            "%s   documented clocking arrangement:\n",
            sens->tag, sens->tag);

    icm20948_aux_probe(sens, "cycled, ODR max, LP_EN=0", ICM20948_I2C_MST_CYCLE, 0x00, false);
    icm20948_aux_probe(sens, "cycled, ODR max, LP_EN=1", ICM20948_I2C_MST_CYCLE, 0x00, true);
    icm20948_aux_probe(sens, "cycled, ODR/16, LP_EN=1", ICM20948_I2C_MST_CYCLE, 0x04, true);
    icm20948_aux_probe(sens, "continuous, LP_EN=0", 0x00, 0x00, false);
    icm20948_aux_probe(sens, "continuous, LP_EN=1", 0x00, 0x00, true);

    fprintf(stderr,
            "%s   If none worked, the auxiliary bus is held low or the AK09916 is not\n"
            "%s   powered. If one worked, that arrangement is the fix.\n",
            sens->tag, sens->tag);
}

bool icm20948_mag_available(icm20948_handle_t sensor)
{
    const icm20948_dev_t* sens = sensor;
    return sens != NULL && sens->mag_enabled;
}

int icm20948_mag_init(icm20948_handle_t sensor, icm20948_mag_mode_t mode, bool debug)
{
    int ret;
    unsigned attempt;
    uint8_t tmp;
    uint8_t wia[2] = {0, 0};
    bool identified = false;
    icm20948_dev_t* sens = sensor;

    sens->mag_enabled = false;
    sens->aux_status = 0;

    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;

    // Disconnect the auxiliary bus from the host pins before the master is
    // enabled: with pass-through on, the master drives nothing.
    ret = icm20948_set_bypass(sensor, false);
    if (ret != 0)
        return ret;

    // Duty cycle the master off the sample clock and run it at the full 1.1 kHz
    // base rate, so a configuration transaction completes in about a
    // millisecond rather than waiting on the (much slower) output data rate.
    ret = icm20948_set_bank(sensor, 3);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_MST_ODR_CFG, 0x00);
    if (ret != 0)
        return ret;
    // 0x17: bit 4 sends a stop between reads rather than a repeated start,
    // which the AK09916 needs; the low nibble selects the 345.6 kHz bus clock.
    ret = icm20948_write_spi(sensor, ICM20948_I2C_MST_CTRL, 0x17);
    if (ret != 0)
        return ret;

    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_LP_CONFIG, ICM20948_I2C_MST_CYCLE);
    if (ret != 0)
        return ret;

    ret = icm20948_read_spi(sensor, ICM20948_USER_CTRL, &tmp, 1);
    if (ret != 0)
        return ret;
    // I2C_IF_DIS is already set by icm20948_configure and preserved here: the
    // primary I2C slave interface owns pins the auxiliary master needs.
    ret = icm20948_write_spi(sensor, ICM20948_USER_CTRL, (uint8_t)(tmp | ICM20948_I2C_MST_EN));
    if (ret != 0)
        return ret;
    usleep(ICM20948_MAG_XFER_WAIT_US);

    // Soft reset the magnetometer so it starts from a known state. A failure
    // here is not fatal: the part may be mid-transaction from a previous run,
    // which is exactly what the retry loop below recovers from.
    (void)icm20948_mag_write(sensor, ICM20948_MAG_CNTL_3, 0x01);
    usleep(ICM20948_MAG_RESET_WAIT_US);

    for (attempt = 0; attempt < ICM20948_MAG_START_ATTEMPTS; attempt++)
    {
        if (icm20948_mag_read(sensor, ICM20948_MAG_WIA_1, &wia[0]) == 0 &&
            icm20948_mag_read(sensor, ICM20948_MAG_WIA_2, &wia[1]) == 0 &&
            wia[0] == ICM20948_MAG_WIA_1_VAL && wia[1] == ICM20948_MAG_WIA_2_VAL)
        {
            identified = true;
            break;
        }

        // Clear a wedged transfer and try again. This recovers the common case
        // of a previous run being killed with a channel still armed.
        ret = icm20948_aux_master_reset(sensor);
        if (ret != 0)
            return ret;
    }

    if (!identified)
    {
        fprintf(stderr, "%s Magnetometer id mismatch, expected 0x%02X 0x%02X but read 0x%02X 0x%02X after %u tries\n",
                sens->tag, ICM20948_MAG_WIA_1_VAL, ICM20948_MAG_WIA_2_VAL, wia[0], wia[1],
                ICM20948_MAG_START_ATTEMPTS);
        icm20948_mag_report_failure(sens, debug);
        return -1;
    }

    fprintf(stdout, "%s Magnetometer AK09916 detected (0x%02X 0x%02X)\n", sens->tag, wia[0], wia[1]);

    ret = icm20948_mag_write(sensor, ICM20948_MAG_CNTL_2, (uint8_t)mode);
    if (ret != 0)
    {
        fprintf(stderr, "%s Magnetometer mode set failed!\n", sens->tag);
        return ret;
    }
    usleep(ICM20948_MAG_XFER_WAIT_US);

    // Arm SLV0 for the streaming read. Starting at ST1 and taking 9 bytes
    // covers ST1, the six data bytes and ST2. ST2 must be included: the
    // magnetometer only releases its output registers for the next measurement
    // once ST2 has been read, so omitting it stalls the data at one sample.
    ret = icm20948_set_bank(sensor, 3);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV0_ADDR, ICM20948_MAG_ADDRESS | ICM20948_MAG_READ);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV0_REG, ICM20948_MAG_STATUS_1);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(sensor, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLVX_EN | ICM20948_MAG_DATA_LEN);
    if (ret != 0)
        return ret;

    ret = icm20948_set_bank(sensor, 0);
    if (ret != 0)
        return ret;

    sens->mag_enabled = true;
    return 0;
}

// Decode the 9 magnetometer bytes lifted out of EXT_SLV_SENS_DATA.
static void icm20948_parse_mag(icm20948_dev_t* sens, const uint8_t* ext)
{
    const uint8_t st1 = ext[0];
    const uint8_t st2 = ext[8];

    if ((st1 & ICM20948_MAG_DRDY) == 0)
    {
        // No new measurement since the last read. The magnetometer runs at
        // 100 Hz while this is polled slightly faster, so this is routine --
        // the previous sample stays in place and is still valid.
        return;
    }

    if ((st2 & ICM20948_MAG_OVF) != 0)
    {
        // Field exceeded the sensor range; the sample is meaningless.
        sens->data->mag_valid = false;
        return;
    }

    // Magnetometer output is little endian, unlike the big endian accel and
    // gyro registers on the main die.
    const int16_t raw_x = (int16_t)((uint16_t)ext[2] << 8 | ext[1]);
    const int16_t raw_y = (int16_t)((uint16_t)ext[4] << 8 | ext[3]);
    const int16_t raw_z = (int16_t)((uint16_t)ext[6] << 8 | ext[5]);

    // Axis alignment. The magnetometer die is mounted rotated 180 degrees about
    // X relative to the accelerometer and gyroscope, so Y and Z are inverted.
    // This matches the compass mounting matrix diag(1, -1, -1) in InvenSense's
    // own ICM-20948 driver. Getting this wrong yields a heading that looks
    // plausible while sitting flat but goes wrong as soon as the board tilts.
    sens->data->mx_raw = raw_x;
    sens->data->my_raw = (int16_t)-raw_y;
    sens->data->mz_raw = (int16_t)-raw_z;

    sens->data->mx = (float)sens->data->mx_raw * ICM20948_MAG_SENSITIVITY;
    sens->data->my = (float)sens->data->my_raw * ICM20948_MAG_SENSITIVITY;
    sens->data->mz = (float)sens->data->mz_raw * ICM20948_MAG_SENSITIVITY;
    sens->data->mag_valid = true;
}

float icm20948_get_acce_sensitivity(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = sensor;
    icm20948_acce_fs_t acce_fs = sens->acce_fs;
    switch (acce_fs)
    {
    case ACCE_FS_2G:
        return 16384;
    case ACCE_FS_4G:
        return 8192;
    case ACCE_FS_8G:
        return 4096;
    case ACCE_FS_16G:
        return 2048;
    }
    return 16384;
}

float icm20948_get_gyro_sensitivity(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = sensor;
    icm20948_gyro_fs_t gyro_fs = sens->gyro_fs;
    switch (gyro_fs)
    {
    case GYRO_FS_250DPS:
        return 131;
    case GYRO_FS_500DPS:
        return 65.5f;
    case GYRO_FS_1000DPS:
        return 32.8f;
    case GYRO_FS_2000DPS:
        return 16.4f;
    }
    return 131;
}

int icm20948_get_gyro(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = (icm20948_dev_t*)sensor;
    uint8_t data_rd[6];
    float gyro_sensitivity = icm20948_get_gyro_sensitivity(sensor);
    if (sens->bank != 0)
    {
        icm20948_set_bank(sensor, 0);
    }
    int ret = icm20948_read_spi(sensor, ICM20948_GYRO_OUT, data_rd, sizeof(data_rd));
    if (ret != 0)
    {
        return ret;
    }

    sens->data->gx_raw = (int16_t)((uint16_t)data_rd[0] << 8 | data_rd[1]);
    sens->data->gy_raw = (int16_t)((uint16_t)data_rd[2] << 8 | data_rd[3]);
    sens->data->gz_raw = (int16_t)((uint16_t)data_rd[4] << 8 | data_rd[5]);

    sens->data->gx = (float)sens->data->gx_raw / gyro_sensitivity - sens->data->gx_bias;
    sens->data->gy = (float)sens->data->gy_raw / gyro_sensitivity - sens->data->gy_bias;
    sens->data->gz = (float)sens->data->gz_raw / gyro_sensitivity - sens->data->gz_bias;

    return ret;
}

// Convert the raw temperature word and fold it into the running average.
static void icm20948_parse_temp(icm20948_dev_t* sens, const uint8_t* data_rd)
{
    const int16_t temp_raw = (int16_t)((uint16_t)data_rd[0] << 8 | data_rd[1]);
    const float temp = (float)temp_raw / 333.87f + 21.0f;

    if (!sens->temp_valid)
    {
        // Seed the filter with the first reading, otherwise it spends its first
        // few seconds creeping up from zero.
        sens->data->temp = temp;
        sens->temp_valid = true;
        return;
    }
    sens->data->temp = 0.9f * sens->data->temp + 0.1f * temp;
}

int icm20948_get_agmt(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = sensor;
    uint8_t data_rd[ICM20948_AGMT_LEN];
    const uint8_t len = sens->mag_enabled ? ICM20948_AGMT_LEN : ICM20948_AGT_LEN;

    if (sens->bank != 0)
    {
        icm20948_set_bank(sensor, 0);
    }

    // ACCEL_OUT .. EXT_SLV_SENS_DATA_08 is one contiguous block, so a single
    // read covers every sensor on the part.
    const int ret = icm20948_read_spi(sensor, ICM20948_ACCEL_OUT, data_rd, len);
    if (ret != 0)
    {
        return ret;
    }

    const float acce_sensitivity = icm20948_get_acce_sensitivity(sensor);
    const float gyro_sensitivity = icm20948_get_gyro_sensitivity(sensor);

    sens->data->ax_raw = (int16_t)((uint16_t)data_rd[0] << 8 | data_rd[1]);
    sens->data->ay_raw = (int16_t)((uint16_t)data_rd[2] << 8 | data_rd[3]);
    sens->data->az_raw = (int16_t)((uint16_t)data_rd[4] << 8 | data_rd[5]);
    sens->data->gx_raw = (int16_t)((uint16_t)data_rd[6] << 8 | data_rd[7]);
    sens->data->gy_raw = (int16_t)((uint16_t)data_rd[8] << 8 | data_rd[9]);
    sens->data->gz_raw = (int16_t)((uint16_t)data_rd[10] << 8 | data_rd[11]);

    sens->data->ax = (float)sens->data->ax_raw / acce_sensitivity;
    sens->data->ay = (float)sens->data->ay_raw / acce_sensitivity;
    sens->data->az = (float)sens->data->az_raw / acce_sensitivity;

    sens->data->gx = (float)sens->data->gx_raw / gyro_sensitivity - sens->data->gx_bias;
    sens->data->gy = (float)sens->data->gy_raw / gyro_sensitivity - sens->data->gy_bias;
    sens->data->gz = (float)sens->data->gz_raw / gyro_sensitivity - sens->data->gz_bias;

    icm20948_parse_temp(sens, &data_rd[12]);

    if (sens->mag_enabled)
    {
        icm20948_parse_mag(sens, &data_rd[ICM20948_AGT_LEN]);
    }

    return 0;
}

int icm20948_calibrate_gyro(icm20948_handle_t sensor, int samples, int interval_us)
{
    icm20948_dev_t* sens = sensor;

    if (samples <= 0)
    {
        return -1;
    }

    // Measure the raw rate, so clear any previously stored offset first.
    sens->data->gx_bias = 0.0f;
    sens->data->gy_bias = 0.0f;
    sens->data->gz_bias = 0.0f;
    sens->data->gyro_calibrated = false;

    double sum[3] = {0.0, 0.0, 0.0};
    float lo[3] = {0.0f, 0.0f, 0.0f};
    float hi[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < samples; i++)
    {
        const int ret = icm20948_get_gyro(sensor);
        if (ret != 0)
        {
            return ret;
        }

        const float sample[3] = {sens->data->gx, sens->data->gy, sens->data->gz};
        for (int axis = 0; axis < 3; axis++)
        {
            sum[axis] += sample[axis];
            if (i == 0 || sample[axis] < lo[axis])
                lo[axis] = sample[axis];
            if (i == 0 || sample[axis] > hi[axis])
                hi[axis] = sample[axis];
        }

        if (interval_us > 0)
        {
            usleep((useconds_t)interval_us);
        }
    }

    // A stationary gyro produces a tight cluster around its offset. A wide
    // spread means the board moved, and averaging that would bake the motion
    // into the offset -- far worse than having no calibration at all.
    for (int axis = 0; axis < 3; axis++)
    {
        const float spread = hi[axis] - lo[axis];
        if (spread > ICM20948_GYRO_CAL_MAX_SPREAD)
        {
            fprintf(stderr, "%s Gyro calibration failed: axis %d moved %.2f deg/s, keep the board still.\n", sens->tag,
                    axis, (double)spread);
            return -1;
        }
    }

    sens->data->gx_bias = (float)(sum[0] / samples);
    sens->data->gy_bias = (float)(sum[1] / samples);
    sens->data->gz_bias = (float)(sum[2] / samples);
    sens->data->gyro_calibrated = true;

    return 0;
}

int icm20948_check_online(icm20948_handle_t sensor)
{
    icm20948_dev_t* sens = sensor;
    uint8_t device_id;
    if (icm20948_get_deviceid(sensor, &device_id) != 0)
    {
        return -1;
    }
    if (device_id != ICM20948_WHO_AM_I_VAL)
    {
        fprintf(stderr, "%s Device offline!\n", sens->tag);
        return -1;
    }
    return 0;
}

int icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs)
{
    int ret;
    uint8_t tmp;
    icm20948_dev_t* sens = sensor;

    ret = icm20948_set_bank(sensor, 2);
    if (ret != 0)
        return ret;

    ret = icm20948_read_spi(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);
    if (ret != 0)
        return ret;

    // Clear only GYRO_FS_SEL (bits 2:1). The previous mask here was 0x09, which
    // also wiped the DLPFCFG field, silently undoing any low pass filter set
    // beforehand.
    tmp &= (uint8_t)~0x06;
    tmp |= (uint8_t)(gyro_fs << 1);

    ret = icm20948_write_spi(sensor, ICM20948_GYRO_CONFIG_1, tmp);
    if (ret != 0)
    {
        fprintf(stderr, "%s Set gyro fs failed!\n", sens->tag);
        return ret;
    }
    // if set gyro fs success, record to sensor
    sens->gyro_fs = gyro_fs;
    return icm20948_set_bank(sensor, 0);
}

int icm20948_set_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t acce_fs)
{
    int ret;
    uint8_t tmp;
    icm20948_dev_t* sens = sensor;

    ret = icm20948_set_bank(sensor, 2);
    if (ret != 0)
        return ret;

    ret = icm20948_read_spi(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);
    if (ret != 0)
        return ret;

    // Clear only ACCEL_FS_SEL (bits 2:1); see the note in icm20948_set_gyro_fs.
    tmp &= (uint8_t)~0x06;
    tmp |= (uint8_t)(acce_fs << 1);

    ret = icm20948_write_spi(sensor, ICM20948_ACCEL_CONFIG, tmp);
    if (ret != 0)
    {
        fprintf(stderr, "%s Set acce fs failed!\n", sens->tag);
        return ret;
    }
    // if set acce fs success, record to sensor
    sens->acce_fs = acce_fs;
    return icm20948_set_bank(sensor, 0);
}

int icm20948_set_acce_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_acce)
{
    int ret;
    uint8_t tmp;

    ret = icm20948_set_bank(sensor, 2);
    if (ret != 0)
        return -1;

    ret = icm20948_read_spi(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);
    if (ret != 0)
        return -1;

    // DLPFCFG occupies bits 5:3, and bit 0 (FCHOICE) selects whether the filter
    // is in circuit at all. Owning both here keeps "off" a single coherent
    // request rather than something the caller must arrange in two steps.
    tmp &= 0xC6;
    if (dlpf_acce != ICM20948_DLPF_OFF)
    {
        tmp |= (uint8_t)(dlpf_acce << 3) | 0x01;
    }

    ret = icm20948_write_spi(sensor, ICM20948_ACCEL_CONFIG, tmp);
    if (ret != 0)
        return -1;

    return icm20948_set_bank(sensor, 0);
}

int icm20948_set_gyro_dlpf(icm20948_handle_t sensor, icm20948_dlpf_t dlpf_gyro)
{
    int ret;
    uint8_t tmp;

    ret = icm20948_set_bank(sensor, 2);
    if (ret != 0)
        return -1;

    ret = icm20948_read_spi(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);
    if (ret != 0)
        return -1;

    // See the note in icm20948_set_acce_dlpf: bits 5:3 select the cutoff and
    // bit 0 puts the filter in circuit.
    tmp &= 0xC6;
    if (dlpf_gyro != ICM20948_DLPF_OFF)
    {
        tmp |= (uint8_t)(dlpf_gyro << 3) | 0x01;
    }

    ret = icm20948_write_spi(sensor, ICM20948_GYRO_CONFIG_1, tmp);
    if (ret != 0)
        return -1;

    return icm20948_set_bank(sensor, 0);
}

int icm20948_configure(icm20948_handle_t icm20948, const icm20948_config_t* config)
{
    int ret;
    icm20948_dev_t* sens = icm20948;
    icm20948_config_t defaults;

    if (config == NULL)
    {
        icm20948_config_default(&defaults);
        config = &defaults;
    }

    ret = icm20948_reset(icm20948);
    if (ret != 0)
    {
        fprintf(stderr, "%s Reset failed!\n", sens->tag);
        return ret;
    }

    // The reset sequence takes tens of milliseconds; reading back too early
    // returns whatever the part happened to have on the bus.
    usleep(100000);

    ret = icm20948_wake_up(icm20948);
    if (ret != 0)
    {
        fprintf(stderr, "%s Wake up failed!\n", sens->tag);
        return ret;
    }
    usleep(20000);

    ret = icm20948_set_bank(icm20948, 0);
    if (ret != 0)
    {
        fprintf(stderr, "%s Set bank failed!\n", sens->tag);
        return ret;
    }

    // The part auto-detects its host interface and the reset above cleared that
    // choice. Pin it to SPI immediately: until the I2C slave interface is
    // disabled it can be re-selected by bus noise, and it also owns pins the
    // auxiliary I2C master needs for the magnetometer.
    uint8_t user_ctrl;
    ret = icm20948_read_spi(icm20948, ICM20948_USER_CTRL, &user_ctrl, 1);
    if (ret != 0)
        return ret;
    ret = icm20948_write_spi(icm20948, ICM20948_USER_CTRL, (uint8_t)(user_ctrl | ICM20948_I2C_IF_DIS));
    if (ret != 0)
        return ret;

    uint8_t device_id;
    ret = icm20948_get_deviceid(icm20948, &device_id);
    if (ret != 0)
    {
        fprintf(stderr, "%s Get device id failed!\n", sens->tag);
        return ret;
    }
    fprintf(stdout, "%s Device ID:0x%02X\n", sens->tag, device_id);
    if (device_id != ICM20948_WHO_AM_I_VAL)
    {
        fprintf(stderr, "%s Device id mismatch!\n", sens->tag);
        return -1;
    }

    // Magnetometer before the accel and gyro are reconfigured. The auxiliary
    // I2C master shares the sample clock with them, and bringing it up against
    // the post-reset rates rather than the ones set below matches the order the
    // reference driver uses.
    if (config->enable_mag)
    {
        ret = icm20948_mag_init(icm20948, config->mag_mode, config->mag_debug);
        if (ret != 0)
        {
            if (config->mag_required)
                return ret;
            fprintf(stderr,
                    "%s Continuing without the magnetometer. Roll and pitch stay absolute,\n"
                    "%s but yaw is gyro-only and will drift.\n",
                    sens->tag, sens->tag);
        }
    }

    // Filters first, full scale ranges second: they share registers, and the
    // range setters preserve the filter bits rather than the other way round.
    ret = icm20948_set_gyro_dlpf(icm20948, config->gyro_dlpf);
    if (ret != 0)
    {
        return ret;
    }

    ret = icm20948_set_acce_dlpf(icm20948, config->acce_dlpf);
    if (ret != 0)
    {
        return ret;
    }

    ret = icm20948_set_gyro_fs(icm20948, config->gyro_fs);
    if (ret != 0)
    {
        return ret;
    }

    ret = icm20948_set_acce_fs(icm20948, config->acce_fs);
    if (ret != 0)
    {
        return ret;
    }

    ret = icm20948_set_sample_rate_div(icm20948, config->sample_rate_div);
    if (ret != 0)
    {
        fprintf(stderr, "%s Set sample rate failed!\n", sens->tag);
        return ret;
    }

    return icm20948_set_bank(icm20948, 0);
}

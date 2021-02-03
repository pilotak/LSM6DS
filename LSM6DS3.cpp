/*
MIT License

Copyright (c) 2021 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "mbed.h"
#include "LSM6DS3.h"

LSM6DS3::LSM6DS3(int address):
    LSM6DS{address} {
}

LSM6DS3::LSM6DS3(PinName sda, PinName scl, int address, uint32_t frequency):
    LSM6DS{sda, scl, address, frequency} {
}

bool LSM6DS3::init(I2C *i2c_obj) {
    if (!LSM6DS::init(i2c_obj)) {
        return false;
    }

    if (!checkWhoAmI(LSM6DS3_WHOAMI)) {
        tr_error("Different chip ID");
        return false;
    }

    // as we don't know current setting
    if (!reset()) {
        return false;
    }

    if (!updateGyroScale()) {
        return false;
    }

    return updateAccelScale();
}

bool LSM6DS3::setupAccel(lsm6ds3_accel_odr_t odr, lsm6ds3_accel_scale_t scale, lsm6ds3_accel_aa_filter_t filter) {
    char data[1];

    // set XL_BW_SCAL_ODR=1 otherwise out new filter will not be used
    if (!readRegister(REG_CTRL4_C, data)) {
        return false;
    }

    data[0] |= 0b10000000;

    if (!writeRegister(REG_CTRL4_C, data)) {
        return false;
    }

    if (!LSM6DS::setupAccel((char)odr, (char)scale, (char)filter)) {
        return false;
    }

    return updateAccelScale();
}

bool LSM6DS3::setAccelFilter(lsm6ds3_accel_lhpf_t filter, bool lp_6d) {
    char data[1];

    if (!readRegister(REG_TAP_CFG, data)) {
        return false;
    }

    if (filter == AccelSlopeFilter) {
        if (data[0] & 0b10000) {
            data[0] &= ~0b10000; // SLOPE_FDS

            tr_info("Turning off SLOPE_FDS");

            if (!writeRegister(REG_TAP_CFG, data)) {
                return false;
            }
        }

    } else {
        if (!(data[0] & 0b10000)) {
            data[0] |= 0b10000; // SLOPE_FDS

            tr_info("Turning on SLOPE_FDS");

            if (!writeRegister(REG_TAP_CFG, data)) {
                return false;
            }
        }
    }

    if (!readRegister(REG_CTRL8_XL, data)) {
        return false;
    }

    if (filter == AccelFilter_Off) {
        data[0] &= ~0b00000100; // HP_SLOPE_XL_EN

    } else {
        data[0] |= 0b00000100; // HP_SLOPE_XL_EN

        if (filter >= AccelLPF2_0) {
            data[0] |= 0b10000000; // LPF2_XL_EN
            data[0] |= (((char)filter - 4) << 5); // HPCF_XL

            tr_info("Enabling LPF2");

        } else {
            data[0] &= ~0b11100000; // LPF2_XL_EN & HPCF_XL
            data[0] |= ((char)filter << 5); // HPCF_XL

            tr_info("Enabling %s", filter == AccelSlopeFilter ? "slope control" : "HPF");
        }
    }

    data[0] &= ~0b00000001; // LOW_PASS_ON_6D

    if (filter == AccelFilter_Off || filter >= AccelLPF2_0) {
        tr_info("Low-pass filter on 6D");
        data[0] |= lp_6d;

    } else if (lp_6d) {
        tr_error("LPF on 6D needs AccelLPF2 filter set or AccelFilter_Off");
    }

    return writeRegister(REG_CTRL8_XL, data);
}

bool LSM6DS3::setGyroFilter(lsm6ds3_gyro_hpf_t filter) {
    return LSM6DS::setGyroFilter((char)filter, (filter == GyroHPF_Off));
}

bool LSM6DS3::updateAccelScale() {
    char data[1];

    if (!readRegister(REG_CTRL1_XL, data)) {
        return false;
    }

    switch ((data[0] >> 2) & 0b11) {
        case AccelScale_2G:
            _accel_scale = 2;
            break;

        case AccelScale_16G:
            _accel_scale = 16;
            break;

        case AccelScale_4G:
            _accel_scale = 4;
            break;

        case AccelScale_8G:
            _accel_scale = 8;
            break;
    }

    return true;
}

bool LSM6DS3::significantMotion(bool enable, char threshold) {
    char data[1];
    tr_info("Setting up significant motion event");

    if (enable) {
        data[0] = 0b10000000;

        if (!writeRegister(REG_FUNC_CFG_ACCESS, data)) {
            return false;
        }

        data[0] = threshold;

        if (!writeRegister(REG_SM_THS, data)) {
            return false;
        }

        data[0] = 0;

        if (!writeRegister(REG_FUNC_CFG_ACCESS, data)) {
            return false;
        }
    }

    if (!readRegister(REG_CTRL10_C, data)) {
        return false;
    }

    data[0] &= ~0b101;
    data[0] |= (char)enable; // SIGN_MOTION_EN
    data[0] |= 0b100; // FUNC_EN

    if (!writeRegister(REG_CTRL10_C, data)) {
        return false;
    }

    return true;
}

bool LSM6DS3::setWakeup(char threshold, char wake_duration, char sleep_duration) {
    char data[1];
    tr_info("Setting up wakeup event");

    if (!readRegister(REG_CTRL8_XL, data)) {
        return false;
    }

    if (data[0] & 0b10000000) {
        tr_error("LPF2 must be disabled");
        return false;
    }

    if (!readRegister(REG_WAKE_UP_DUR, data)) {
        return false;
    }

    data[0] &= ~0b01101111;
    data[0] |= sleep_duration & 0b1111; // SLEEP_DUR[3:0]
    data[0] |= (wake_duration & 0b11) << 5; // WAKE_DUR[1:0]

    if (!writeRegister(REG_WAKE_UP_DUR, data)) {
        return false;
    }

    if (!readRegister(REG_WAKE_UP_THS, data)) {
        return false;
    }

    data[0] &= ~0b00111111;
    data[0] |= threshold & 0b111111; // WK_THS[5:0]

    return writeRegister(REG_WAKE_UP_THS, data);
}

bool LSM6DS3::getFnIntReason(char *reason) {
    if (!readRegister((lsm6ds_reg_t)REG_FUNC_SRC, reason, 1)) {
        tr_error("Could not get function interrupt source");
        return false;
    }

    tr_info("Fn interrupt reason - significant motion: %u", (*reason & LSM6DS3_FN_SRC_SIGN_MOTION_IA) >> 6);
    return true;
}

bool LSM6DS3::enableInactivity(bool enable) {
    char data[1];
    tr_info("Setting up inactivity event");

    if (!readRegister(REG_WAKE_UP_THS, data)) {
        return false;
    }

    data[0] &= ~0b01000000;
    data[0] |= (char)enable << 6;

    return writeRegister(REG_WAKE_UP_THS, data);
}

bool LSM6DS3::setIntLatchMode(bool enable) {
    char data[1];

    if (!readRegister(REG_TAP_CFG, data)) {
        return false;
    }

    data[0] &= ~0b00000001; // LIR
    data[0] |= (char)enable;;

    return writeRegister(REG_TAP_CFG, data);
}

bool LSM6DS3::fifoMode(char gyro_decimation, char accel_decimation) {
    char data[1];
    tr_info("Setting FIFO decimation");

    data[0] = 0;
    data[0] |= accel_decimation & 0b111;
    data[0] |= (gyro_decimation & 0b111) << 3;

    return writeRegister((lsm6ds_reg_t)REG_FIFO_CTRL3, data);
}

float LSM6DS3::temperatureToC(int16_t raw) {
    return ((float)raw / 16.0) + 25.0;
}

float LSM6DS3::temperatureToF(int16_t raw) {
    return (temperatureToC(raw) * 9 / 5 + 32);
}

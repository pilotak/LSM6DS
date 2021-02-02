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
#include "LSM6DSO32.h"

LSM6DSO32::LSM6DSO32(int address):
    LSM6DS{address} {
}

LSM6DSO32::LSM6DSO32(PinName sda, PinName scl, int address, uint32_t frequency):
    LSM6DS{sda, scl, address, frequency} {

}

bool LSM6DSO32::init(I2C *i2c_obj) {
    if (!LSM6DS::init(i2c_obj)) {
        return false;
    }

    if (!checkWhoAmI(LSM6DSO32_WHOAMI)) {
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

bool LSM6DSO32::setupAccel(lsm6dso32_accel_odr_t odr, lsm6dso32_accel_scale_t scale,
                           lsm6dso32_accel_lpf2_t lpf2) {
    if (!LSM6DS::setupAccel((char)odr, (char)scale, (char)lpf2)) {
        return false;
    }

    return updateAccelScale();
}
bool LSM6DSO32::setAccelFilter(lsm6dso32_accel_lhpf_t filter, bool high_pass, bool lp_6d) {
    char data[2];

    if (!readRegister(REG_CTRL8_XL, data)) {
        return false;
    }

    data[0] &= ~0b00000100; // HP_SLOPE_XL_EN

    if (high_pass) {
        tr_info("Accel high-pass filter");
        data[0] |= 0b00000100; // HP_SLOPE_XL_EN

    } else {
        tr_info("Accel low-pass filter");

        if (!readRegister(REG_CTRL1_XL, data + 1)) {
            return false;
        }

        if ((data[1] & 0b10) == 0) {
            tr_error("AccelLFP2_On has to be set first");
            return false;
        }
    }

    data[0] &= ~0b00000001; // LOW_PASS_ON_6D

    if (filter != AccelFilter_Off) {
        data[0] &= ~0b11100000; // HPCF_XL
        data[0] |= ((char)filter << 5); // HPCF_XL

    } else {
        if (!high_pass) {
            tr_info("Enabling low-pass filter on 6D");
            data[0] |= lp_6d;

        } else if (lp_6d) {
            tr_error("LPF on 6D needs low-pass filter set");
        }
    }

    return writeRegister(REG_CTRL8_XL, data);
}

bool LSM6DSO32::setGyroFilter(lsm6dso32_gyro_hpf_t filter) {
    return LSM6DS::setGyroFilter((char)filter, (filter == GyroHPF_Off));
}

bool LSM6DSO32::updateAccelScale() {
    char data[1];

    if (!readRegister(REG_CTRL1_XL, data)) {
        return false;
    }

    switch ((data[0] >> 2) & 0b11) {
        case AccelScale_4G:
            _accel_scale = 4;
            break;

        case AccelScale_32G:
            _accel_scale = 32;
            break;

        case AccelScale_8G:
            _accel_scale = 8;
            break;

        case AccelScale_16G:
            _accel_scale = 16;
            break;
    }

    return true;
}

bool LSM6DSO32::setIntLatchMode(bool enable) {
    // TODO
    /*char data[1];

    if (!readRegister((lsm6ds_reg_t)REG_PAGE_RW, data)) {
        return false;
    }

    data[0] &= ~0b100000000; // EMB_FUNC_LIR
    data[0] |= (char)enable << 6;

    return writeRegister((lsm6ds_reg_t)REG_PAGE_RW, data);*/
    return false;
}

float LSM6DSO32::temperatureToC(int16_t raw) {
    return ((float)raw / 256.0) + 25.0;
}

float LSM6DSO32::temperatureToF(int16_t raw) {
    return (temperatureToC(raw) * 9 / 5 + 32);
}
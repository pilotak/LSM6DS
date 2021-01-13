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

    return check_who_am_i(LSM6DS3_WHOAMI);
}

bool LSM6DS3::set_accel_mode(lsm6ds3_accel_odr_t odr, lsm6ds3_accel_scale_t scale, lsm6ds3_accel_filter_t filter) {
    char data[1];

    // set XL_BW_SCAL_ODR=1 otherwise out new filter will no be used
    if (!readRegiter(REG_CTRL4_C, data, 1)) {
        return false;
    }

    data[0] |= 0b10000000;

    if (!writeRegiter(REG_CTRL4_C, data, 1)) {
        return false;
    }

    return LSM6DS::set_accel_mode((char)odr, (char)scale, (char)filter);
}

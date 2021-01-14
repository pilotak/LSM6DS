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

    return checkWhoAmI(LSM6DSO32_WHOAMI);
}

bool LSM6DSO32::setAccelMode(lsm6dso32_accel_odr_t odr, lsm6dso32_accel_scale_t scale,
                               lsm6dso32_accel_highres_t high_res) {
    return LSM6DS::setAccelMode((char)odr, (char)scale, (char)high_res);
}
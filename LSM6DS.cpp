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
#include "LSM6DS.h"

LSM6DS::LSM6DS(int address):
    _address(address) {
}

LSM6DS::LSM6DS(PinName sda, PinName scl, int address, uint32_t frequency):
    _address(address) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
}

LSM6DS::~LSM6DS(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool LSM6DS::setIntMode(lsm6ds_int_mode_t mode) {
    char data[1];

    if (!readRegister(REG_CTRL3_C, data)) {
        return false;
    }

    data[0] &= ~0b00001000; // clear
    data[0] |= ((char)mode & 0b1) << 4;

    return writeRegister(REG_CTRL3_C, data);
}

bool LSM6DS::getStatus(char *status) {
    if (!readRegister(REG_STATUS, status, 1)) {
        tr_error("Could not get status");
        return false;
    }

    tr_info("New data - accel: %u, gyro: %u, temp: %u", *status & 0b1, (*status & 0b10) >> 1, (*status & 0b100) >> 2);
    return true;
}


bool LSM6DS::setINT1(char reg) {
    return writeRegister(REG_INT1_CTRL, &reg);
}

bool LSM6DS::setINT2(char reg) {
    return writeRegister(REG_INT2_CTRL, &reg);
}

bool LSM6DS::getTemperature(int16_t *raw_temp) {
    char data[2];

    if (!readRegister(REG_OUT_TEMP_L, data, 2)) {
        tr_error("Could not get temperature");
        return false;
    }

    if (raw_temp) {
        *raw_temp = ((int16_t)data[1] << 8) | data[0];
    }

    return true;
}

bool LSM6DS::getAccel(float *x, float *y, float *z) {
    char data[6];
    int16_t raw;
    float calc = _accel_scale / 32768.0;
    tr_info("Getting accel axis");

    if (!readRegister(REG_OUTX_L_XL, data, 6)) {
        tr_error("Could not get accel axis");
        return false;
    }

    if (x) {
        raw = ((int16_t)data[1] << 8) | data[0];
        *x = (raw * calc);
    }

    if (y) {
        raw = ((int16_t)data[3] << 8) | data[2];
        *y = (raw * calc);
    }

    if (z) {
        raw = ((int16_t)data[5] << 8) | data[4];
        *z = (raw * calc);
    }

    return true;
}

bool LSM6DS::getAccel(int16_t *x, int16_t *y, int16_t *z) {
    char data[6];
    tr_info("Getting accel axis");

    if (!readRegister(REG_OUTX_L_XL, data, 6)) {
        tr_error("Could not get accel axis");
        return false;
    }

    if (x) {
        *x = ((int16_t)data[1] << 8) | data[0];
    }

    if (y) {
        *y = ((int16_t)data[3] << 8) | data[2];
    }

    if (z) {
        *z = ((int16_t)data[5] << 8) | data[4];
    }

    return true;
}


bool LSM6DS::getGyro(float *x, float *y, float *z) {
    char data[6];
    int16_t raw;
    float calc = _gyro_scale / 32768.0;
    tr_info("Getting gyro axis");

    if (!readRegister(REG_OUTX_L_G, data, 6)) {
        tr_error("Could not get gyro axis");
        return false;
    }

    if (x) {
        raw = (((int16_t)data[1] << 8) | data[0]);
        *x = (raw * calc);
    }

    if (y) {
        raw = ((int16_t)data[3] << 8) | data[2];
        *y = (raw * calc);
    }

    if (z) {
        raw = ((int16_t)data[5] << 8) | data[4];
        *z = (raw * calc);
    }

    return true;
}

bool LSM6DS::getGyro(int16_t *x, int16_t *y, int16_t *z) {
    char data[6];
    tr_info("Getting gyro axis");

    if (!readRegister(REG_OUTX_L_G, data, 6)) {
        tr_error("Could not get gyro axis");
        return false;
    }

    if (x) {
        *x = ((int16_t)data[1] << 8) | data[0];
    }

    if (y) {
        *y = ((int16_t)data[3] << 8) | data[2];
    }

    if (z) {
        *z = ((int16_t)data[5] << 8) | data[4];
    }

    return true;
}

bool LSM6DS::reset() {
    char data[1];

    if (!readRegister(REG_CTRL3_C, data)) {
        return false;
    }

    data[0] |= 1;

    return writeRegister(REG_CTRL3_C, data);
}

bool LSM6DS::init(I2C *i2c_obj) {
    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    return true;
}

bool LSM6DS::setupAccel(char odr_xl, char fs_xl, char bw_xl) {
    char data[2];

    tr_info("Setting new accelerometer mode");

    data[0] = (bw_xl & 0b11);
    data[0] |= ((fs_xl & 0b11) << 2);
    data[0] |= ((odr_xl & 0b1111) << 4);

    if (!writeRegister(REG_CTRL1_XL, data)) {
        return false;
    }

    if (!readRegister(REG_CTRL1_XL, data + 1)) {
        return false;
    }

    return (data[0] == data[1]);
}

bool LSM6DS::setupGyro(lsm6ds_gyro_odr_t odr, lsm6ds_gyro_scale_t scale, bool fs_125) {
    char data[2];

    tr_info("Setting new gyroscope mode");

    data[0] = ((char)fs_125 << 1) | 0;
    data[0] |= ((char)(scale & 0b11) << 2);
    data[0] |= (((char)odr & 0b1111) << 4);

    if (!writeRegister(REG_CTRL2_G, data)) {
        return false;
    }

    if (!readRegister(REG_CTRL2_G, data + 1)) {
        return false;
    }

    if (data[0] != data[1]) {
        return false;
    }

    return updateGyroScale(data + 1);
}

bool LSM6DS::setGyroMode(bool high_performance) {
    char data[1];

    if (!readRegister(REG_CTRL7_G, data)) {
        return false;
    }

    data[0] &= ~0b10000000;
    data[0] |= (char)(!high_performance) << 7;

    return writeRegister(REG_CTRL7_G, data);
}

bool LSM6DS::setGyroFilter(char filter, bool enable) {
    char data[1];

    if (!readRegister(REG_CTRL7_G, data)) {
        return false;
    }

    data[0] &= ~0b01000000; // HP_G_EN

    if (enable) {
        data[0] |= 0b01000000; // HP_G_EN
        data[0] &= ~0b00110000;
        data[0] |= ((filter & 0b11) << 4); // HPCF_G
    }

    return writeRegister(REG_CTRL7_G, data);
}

bool LSM6DS::updateGyroScale(char *reg_ctrl2_g) {
    char data[1];

    if (reg_ctrl2_g == nullptr) {
        if (!readRegister(REG_CTRL2_G, data)) {
            return false;
        }

    } else {
        memcpy(data, reg_ctrl2_g, 1);
    }

    switch ((data[0] >> 2) & 0b11) {
        case GyroScale_250DPS:
            _gyro_scale = 250;
            break;

        case GyroScale_500DPS:
            _gyro_scale = 500;
            break;

        case GyroScale_1000DPS:
            _gyro_scale = 1000;
            break;

        case GyroScale_2000DPS:
            _gyro_scale = 2000;
            break;
    }

    return true;
}

bool LSM6DS::checkWhoAmI(uint8_t id) {
    char data[1];
    tr_info("Checking device ID");

    if (!readRegister(REG_WHO_AM_I, data, sizeof(data))) {
        return false;
    }

    return (data[0] == id);
}

bool LSM6DS::writeRegister(lsm6ds_reg_t reg, const char *data, size_t len) {
    tr_debug("Write register: %02X", reg);

    if (data == nullptr || len == 0) {
        return false;
    }

    char *d = new char[len + 1];

    if (!d) {
        tr_error("No memory");
        return false;
    }

    d[0] = (char)reg;
    memcpy(d + 1, data, len);

    _i2c->lock();

    if (!write(d, len + 1)) {
        tr_error("Error writing data");
        goto ERROR;
    }

    _i2c->unlock();
    return true;

ERROR:
    _i2c->unlock();
    return false;
}

bool LSM6DS::readRegister(lsm6ds_reg_t reg, char *data, size_t len) {
    tr_debug("Read register: %02X", reg);

    if (data == nullptr || len == 0) {
        return false;
    }

    data[0] = (char)reg;

    _i2c->lock();

    if (!write(data, 1, false)) {
        tr_error("Error writing data");
        goto ERROR;
    }

    if (!read(data, len)) {
        tr_error("Couldn't read register");
        goto ERROR;
    }

    _i2c->unlock();
    return true;

ERROR:
    _i2c->unlock();
    return false;
}

bool LSM6DS::read(char *buffer, size_t len) {
    int32_t ack = -1;

    ack = _i2c->read(_address, buffer, len);

    if (ack != 0) {
        return false;
    }

    tr_debug("Read data(%u): %s", len, tr_array(reinterpret_cast<uint8_t *>(buffer), len));

    return true;
}

bool LSM6DS::write(const char *data, size_t len, bool stop) {
    tr_debug("Sending data[%u]: %s", len, tr_array(reinterpret_cast<const uint8_t *>(data), len));

    int32_t ack = _i2c->write(_address, data, len, !stop);

    if (ack != 0) {
        tr_error("Write failed");
        return false;
    }

    return true;
}
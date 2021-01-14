/*
MIT License

Copyright (c) 2020 Pavel Slama

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

#ifndef LSM6DS_H
#define LSM6DS_H

#include <climits>
#include <chrono>
#include "mbed.h"

using namespace std::chrono;

#include "mbed-trace/mbed_trace.h"
#ifndef TRACE_GROUP
    #define TRACE_GROUP "6DOF"
#endif

#define LSM6DS_DEFAULT_ADDRESS (0x6B << 1)

class LSM6DS {
  public:
    typedef enum {
        GyroODR_PowerDown = 0,
        GyroODR_12_5Hz, // low power
        GyroODR_26Hz,   // low power
        GyroODR_52Hz,   // low power
        GyroODR_104Hz,  // normal mode
        GyroODR_208Hz,  // normal mode
        GyroODR_416Hz,  // high performance
        GyroODR_833Hz,  // high performance
        GyroODR_1666Hz, // high performance
        GyroODR_3333Hz, // high performance
        GyroODR_6666Hz, // high performance
    } lsm6ds_gyro_odr_t;

    typedef enum {
        GyroScale_250DPS = 0,
        GyroScale_500PS,
        GyroScale_1000DPS,
        GyroScale_2000DPS
    } lsm6ds_gyro_scale_t;

    typedef enum {
        IntMode_PushPull = 0, // default
        IntMode_OpenDrain
    } lsm6ds_int_mode_t;

    LSM6DS(int address);
    LSM6DS(PinName sda, PinName scl, int address, uint32_t frequency = 400000);
    virtual ~LSM6DS(void);

    /**
     * @brief Set the gyroscope mode
     *
     * @param odr Output data rate
     * @param scale Full-scale selection
     * @param fs_125 Full-scale at 125 dps
     * @return true if successful, otherwise false
     */
    bool setGyroMode(lsm6ds_gyro_odr_t odr, lsm6ds_gyro_scale_t scale = GyroScale_250DPS, bool fs_125 = false);

    /**
     * @brief Set the mode of INT1 and INT2 pins
     *
     * @param mode
     * @return true if successful, otherwise false
     */
    bool setIntMode(lsm6ds_int_mode_t mode);

    /**
     * @brief Get status
     * 
     * @param status place to put the reading
     * @return true if successful, otherwise false
     */
    bool getStatus(char *status);

    /**
     * @brief Get the chip temperature
     *
     * @param raw_temp place to put the reading, result in LSB
     * @return true if successful, otherwise false
     */
    bool getTemperature(int16_t *raw_temp);

    /**
     * @brief Perform SW device reset
     *
     * @return true if successful, otherwise false
     */
    bool reset();

  protected:
    typedef enum {
        REG_CTRL1_XL = 0x10,
        REG_CTRL2_G  = 0x11,
        REG_CTRL3_C = 0x12,
        REG_CTRL4_C = 0x13,
        REG_CTRL7_G = 0x16,
        REG_CTRL8_XL = 0x17,
        REG_STATUS = 0x1E,
        REG_OUT_TEMP_L = 0x20,
        REG_OUT_TEMP_H = 0x21,
        REG_OUTX_L_G = 0x22,
        REG_OUTX_H_G = 0x23,
        REG_OUTY_L_G = 0x24,
        REG_OUTY_H_G = 0x25,
        REG_OUTZ_L_G = 0x26,
        REG_OUTZ_H_G = 0x27,
        REG_OUTX_L_XL = 0x28,
        REG_OUTX_H_XL = 0x29,
        REG_OUTY_L_XL = 0x2A,
        REG_OUTY_H_XL = 0x2B,
        REG_OUTZ_L_XL = 0x2C,
        REG_OUTZ_H_XL = 0x2D,
        REG_WHO_AM_I = 0x0F,
    } lsm6ds_reg_t;

    /**
    * @brief Initialise
    *
    * @param i2c_obj pass I2C object if you didn't specify pins in constructor
    * @return true if successful, otherwise false
    */
    bool init(I2C *i2c_obj = nullptr);

    /**
     * @brief Check the device type
     *
     * @param id expected id
     * @return true if successful, otherwise false
     */
    bool checkWhoAmI(uint8_t id);

    /**
     * @brief Set the accelerometer mode
     *
     * @param odr_xl Output data rate and power mode selection
     * @param fs_xl Full-scale selection
     * @param bw_xl Filter selection
     * @return true if successful, otherwise false
     */
    bool setAccelMode(char odr_xl, char fs_xl, char bw_xl);

    /**
     * @brief Write register
     *
     * @param reg register address
     * @param data a pointer to the data block
     * @param len the size of the data to be written
     * @return true if successful, otherwise false
     */
    bool writeRegister(lsm6ds_reg_t reg, const char *data, size_t len = 1);

    /**
     * @brief Read register
     *
     * @param reg register address
     * @param data a pointer to the data block
     * @param len the size of the data to be read
     * @return true if successful, otherwise false
     */
    bool readRegister(lsm6ds_reg_t reg, char *data, size_t len = 1);

  private:
    I2C *_i2c;
    uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
    const int _address = LSM6DS_DEFAULT_ADDRESS;

    /**
     * @brief Main I2C writer function
     *
     * @param data a pointer to the data block
     * @param len the size of the data to be written
     * @param stop whether to send stop command
     * @return true if successful, otherwise false
     */
    bool write(const char *data, size_t len, bool stop = true);

    /**
     * @brief Main I2C reader function
     *
     * @param buffer a place to put the read data
     * @param len size of data to read (make sure it fits into buffer)
     * @return true if successful, otherwise false
     */
    bool read(char *buffer, size_t len);
};

#endif  // LSM6DS_H

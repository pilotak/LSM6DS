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

#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "LSM6DS.h"

#define LSM6DSO32_WHOAMI 0x6C

class LSM6DSO32: public LSM6DS {
  public:
    typedef enum {
        AccelODR_PowerDown = 0,
        AccelODR_12_5Hz, // low power
        AccelODR_26Hz,   // low power
        AccelODR_52Hz,   // low power
        AccelODR_104Hz,  // normal mode
        AccelODR_208Hz,  // normal mode
        AccelODR_416Hz,  // high performance
        AccelODR_833Hz,  // high performance
        AccelODR_1666Hz, // high performance
        AccelODR_3333Hz, // high performance
        AccelODR_6666Hz, // high performance
        AccelODR_1_6Hz,  // low power
    } lsm6dso32_accel_odr_t;

    typedef enum {
        AccelScale_4G = 0,
        AccelScale_32G,
        AccelScale_8G,
        AccelScale_16G
    } lsm6dso32_accel_scale_t;

    typedef enum {
        AccelHighRes_Off = 0b00,
        AccelHighRes_On = 0b10
    } lsm6dso32_accel_highres_t;

    LSM6DSO32(int address);
    LSM6DSO32(PinName sda, PinName scl, int address = LSM6DS_DEFAULT_ADDRESS, uint32_t frequency = 400000);

    /**
    * @brief Initialise the chip
    *
    * @param i2c_obj pass I2C object if you didn't specify pins in constructor
    * @return true if successful, otherwise false
    */
    bool init(I2C *i2c_obj = nullptr);

    /**
     * @brief Set the accelerometer mode
     *
     * @param odr Output data rate and power mode selection
     * @param scale Full-scale selection
     * @param filter High-resolution selection
     * @return true if successful, otherwise false
     */
    bool setAccelMode(lsm6dso32_accel_odr_t odr, lsm6dso32_accel_scale_t scale = AccelScale_4G,
                      lsm6dso32_accel_highres_t high_res = AccelHighRes_Off);

    /**
     * @brief Convert raw temperature reading to °C
     *
     * @param raw reading from getTemperature()
     * @return temperature in °C
     */
    float temperatureToC(int16_t raw);

    /**
     * @brief Convert raw temperature reading to °F
     *
     * @param raw reading from getTemperature()
     * @return temperature in °F
     */
    float temperatureToF(int16_t raw);

  private:
    /**
     * @brief Update lib accelerometer scale
     *
     * @return true if successful, otherwise false
     */
    bool updateAccelScale();
};

#endif

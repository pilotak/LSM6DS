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
    #define TRACE_GROUP "LSM6"
#endif

#define LSM6DS_DEFAULT_ADDRESS (0x6B << 1)

#define LSM6DS_WAKEUP_SRC_FF_IA          0b100000 // free-fall event
#define LSM6DS_WAKEUP_SRC_SLEEP_STATE_IA 0b10000 // inactivity event
#define LSM6DS_WAKEUP_SRC_WU_IA          0b1000 // wakeup event
#define LSM6DS_WAKEUP_SRC_X_WU           0b100 // wakeup event on X-axis
#define LSM6DS_WAKEUP_SRC_Y_WU           0b10 // wakeup event on Y-axis
#define LSM6DS_WAKEUP_SRC_Z_WU           0b1 // wakeup event on Z-axis

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
        GyroScale_500DPS,
        GyroScale_1000DPS,
        GyroScale_2000DPS
    } lsm6ds_gyro_scale_t;

    typedef enum {
        IntMode_PushPull = 0, // default
        IntMode_OpenDrain
    } lsm6ds_int_mode_t;

    typedef enum {
        HighPerformanceEnabled = 0, // default
        HighPerformanceDisabled
    } lsm6ds_hm_mode_t;

    LSM6DS(int address);
    LSM6DS(PinName sda, PinName scl, int address, uint32_t frequency = 400000);
    virtual ~LSM6DS(void);

    /**
     * @brief Setup the gyroscope
     *
     * @param odr output data rate
     * @param scale full-scale selection
     * @param fs_125 full-scale at 125 dps
     * @return true if successful, otherwise false
     */
    bool setupGyro(lsm6ds_gyro_odr_t odr, lsm6ds_gyro_scale_t scale = GyroScale_250DPS, bool fs_125 = false);

    /**
     * @brief Set high-performance operating mode for gyroscope.
     *
     * @param high_performance
     * @return true if successful, otherwise false
     */
    bool setGyroMode(lsm6ds_hm_mode_t high_performance);

    /**
     * @brief Set high-performance operating mode for accelerometer
     *
     * @param high_performance
     * @return true if successful, otherwise false
     */
    bool setAccelMode(lsm6ds_hm_mode_t high_performance);

    /**
     * @brief Enable axis on accelermeter
     *
     * @param x
     * @param y
     * @param z
     * @return true if successful, otherwise false
     */
    bool setAccelAxis(bool x, bool y, bool z);

    /**
     * @brief Enable axis on gyro
     *
     * @param x
     * @param y
     * @param z
     * @return true if successful, otherwise false
     */
    bool setGyroAxis(bool x, bool y, bool z);

    /**
     * @brief INT1 pin control
     *
     * @param reg INT1_CTRL register
     * @return true if successful, otherwise false
     */
    bool setINT1(char reg);

    /**
    * @brief INT2 pin control
    *
    * @param reg INT2_CTRL register
    * @return true if successful, otherwise false
    */
    bool setINT2(char reg);

    /**
     * @brief Functions routing on INT1 register
     *
     * @param reg MD1_CFG register
     * @return true if successful, otherwise false
     */
    bool setFnINT1(char reg);

    /**
     * @brief Functions routing on INT1 register
     *
     * @param reg MD2_CFG register
     * @return true if successful, otherwise false
     */
    bool setFnINT2(char reg);

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
     * @brief Get the Wakeup Reason object
     *
     * @param reason place to put the reading (1 byte)
     * @return true if successful, otherwise false
     */
    bool getWakeupReason(char *reason);

    /**
     * @brief Get the chip temperature
     *
     * @param raw_temp place to put the reading, result in LSB
     * @return true if successful, otherwise false
     */
    bool getTemperature(int16_t *raw_temp);

    /**
     * @brief Get accelerometer axis
     *
     * @param x
     * @param y
     * @param z
     * @return true if successful, otherwise false
     */
    bool getAccel(float *x, float *y, float *z);

    /**
     * @brief Get raw accelerometer axis
     *
     * @param x
     * @param y
     * @param z
     * @return true if successful, otherwise false
     */
    bool getAccel(int16_t *x, int16_t *y, int16_t *z);

    /**
     * @brief Get gyroscope axis
     *
     * @param x pitch
     * @param y roll
     * @param z yaw
     * @return true if successful, otherwise false
     */
    bool getGyro(float *x, float *y, float *z);

    /**
     * @brief Get raw gyroscope axis
     *
     * @param x
     * @param y
     * @param z
     * @return true if successful, otherwise false
     */
    bool getGyro(int16_t *x, int16_t *y, int16_t *z);

    /**
     * @brief Set block data update
     *
     * @param enable
     * @return true if successful, otherwise false
     */
    bool bdu(bool enable);

    /**
     * @brief Perform SW device reset
     *
     * @return true if successful, otherwise false
     */
    bool reset();
  protected:
    typedef enum {
        REG_FUNC_CFG_ACCESS = 0x01,
        REG_WAKE_UP_SRC = 0x1B,
        REG_INT1_CTRL = 0x0D,
        REG_INT2_CTRL = 0x0E,
        REG_WHO_AM_I = 0x0F,
        REG_CTRL1_XL = 0x10,
        REG_CTRL2_G  = 0x11,
        REG_CTRL3_C = 0x12,
        REG_CTRL4_C = 0x13,
        REG_SM_THS = 0x13,
        REG_CTRL6_C = 0x15,
        REG_CTRL7_G = 0x16,
        REG_CTRL8_XL = 0x17,
        REG_CTRL9_XL = 0x18,
        REG_CTRL10_C = 0x19,
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
        REG_WAKE_UP_THS = 0x5B,
        REG_WAKE_UP_DUR = 0x5C,
        REG_MD1_CFG = 0x5E,
        REG_MD2_CFG = 0x5F,
        REG_TAP_CFG = 0x58,
    } lsm6ds_reg_t;
    uint8_t _accel_scale = 4;

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
     * @brief Setup the accelerometer
     *
     * @param odr_xl output data rate and power mode selection
     * @param fs_xl full-scale selection
     * @param bw_xl filter selection
     * @return true if successful, otherwise false
     */
    bool setupAccel(char odr_xl, char fs_xl, char bw_xl);

    /**
     * @brief Set high-pass filter for gyroscope
     *
     * @param filter filter type
     * @param enable
     * @return true if successful, otherwise false
     */
    bool setGyroFilter(char filter, bool enable);

    /**
     * @brief Update lib gyroscope scale
     *
     * @return true if successful, otherwise false
     */
    bool updateGyroScale();

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
     * @param data place to put the reading
     * @param len the size of the data to be read
     * @return true if successful, otherwise false
     */
    bool readRegister(lsm6ds_reg_t reg, char *data, size_t len = 1);

  private:
    I2C *_i2c;
    uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
    const int _address = LSM6DS_DEFAULT_ADDRESS;
    uint16_t _gyro_scale = 250;

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

    bool setAxis(lsm6ds_reg_t reg, bool x, bool y, bool z);
};

#endif  // LSM6DS_H

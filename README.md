# LSM6DS
![build](https://github.com/pilotak/LSM6DS/workflows/build/badge.svg)
[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)

Mbed library for LSM6DS3 & LSM6DSO32 accel &amp; gyro. Supports only I2C

## Accelerometer examples

<details>
<summary>Basic read</summary>

```cpp
#include "mbed.h"
#include "LSM6DS3.h"

#define I2C_SDA PB_9
#define I2C_SCL PB_8

LSM6DS3 accel(I2C_SDA, I2C_SCL);

int main() {
    if (!accel.init()) {
        debug("Could not init\n");
        return 0;
    }

    if (!accel.setupAccel(LSM6DS3::AccelODR_26Hz, LSM6DS3::AccelScale_4G)) {
        debug("Couldn't setup setup slow ODR accel\n");
        return 0;
    }

    while (1) {
        float x, y, z;

        if (!accel.getAccel(&x, &y, &z)) {
            debug("Error reading\n");
            return 0;
        }

        debug("%.6f,%.6f,%.6f\n", x, y, z);

        ThisThread::sleep_for(100ms);
    }
}
```

</details>

<details>
<summary>Significant motion</summary>

```cpp
#include "mbed.h"
#include "LSM6DS3.h"

#define I2C_SDA PB_9
#define I2C_SCL PB_8

EventQueue eQueue(2 * EVENTS_EVENT_SIZE);
LSM6DS3 accel(I2C_SDA, I2C_SCL);
InterruptIn accelInt(PA_9, PullDown);

void accelIntCb() {
    char data[1];

    if (accel.getFnIntReason(data)) {
        if (data[0] & LSM6DS3_FN_SRC_SIGN_MOTION_IA) {
            debug("Significant motion\n");
        }
    }
}

int main() {
    if (!accel.init()) {
        debug("Could not init\n");
        return 0;
    }

    if (!accel.setupAccel(LSM6DS3::AccelODR_26Hz, LSM6DS3::AccelScale_4G)) {
        debug("Couldn't setup setup slow ODR accel\n");
        return 0;
    }

    if (!accel.significantMotion(true, 1)) { // enable, threshold
        debug("Could set setup significant motion\n");
        return 0;
    }

    if (!accel.setINT1(0b01000000)) {
        debug("Could set setup INT\n");
        return 0;
    }

    accelInt.rise(eQueue.event(accelIntCb));

    while (1) {
        eQueue.dispatch_forever();
    }
}
```

</details>

<details>
<summary>Wakeup</summary>

```cpp
#include "mbed.h"
#include "LSM6DS3.h"

#define I2C_SDA PB_9
#define I2C_SCL PB_8

EventQueue eQueue(2 * EVENTS_EVENT_SIZE);
LSM6DS3 accel(I2C_SDA, I2C_SCL);
InterruptIn accelInt(PA_9, PullDown);

void accelIntCb() {
    char data[1];

    if (accel.getWakeupReason(data)) {
        if (data[0] & LSM6DS_WAKEUP_SRC_WU_IA) {
            debug("Wake up event\n");
        }
    }
}

int main() {
    if (!accel.init()) {
        debug("Could not init\n");
        return 0;
    }

    if (!accel.setupAccel(LSM6DS3::AccelODR_52Hz, LSM6DS3::AccelScale_4G)) {
        debug("Couldn't setup setup slow ODR accel\n");
        return 0;
    }

    if (!accel.setAccelFilter(LSM6DS3::AccelSlopeFilter)) {
        debug("Couldn't setup filter\n");
        return 0;
    }

    if (!accel.setWakeup(2, 2, 0)) { // threshold, wakeup time, sleep time
        debug("Couldn't setup wakeup\n");
        return 0;
    }

    if (!accel.setFnINT1(0b00100000)) {
        debug("Couldn't setup setup INT\n");
        return 0;
    }

    accelInt.rise(eQueue.event(accelIntCb));

    while (1) {
        eQueue.dispatch_forever();
    }
}
```

</details>

<details>
<summary>In/activity</summary>

```cpp
#include "mbed.h"
#include "LSM6DS3.h"

#define I2C_SDA PB_9
#define I2C_SCL PB_8

EventQueue eQueue(2 * EVENTS_EVENT_SIZE);
LSM6DS3 accel(I2C_SDA, I2C_SCL);
InterruptIn accelInt(PA_9, PullDown);

void accelIntCb() {
    char data[1];

    if (accel.getWakeupReason(data)) {
        if (data[0] & LSM6DS_WAKEUP_SRC_SLEEP_STATE_IA) {
            if (data[0] & 0b111) {
                debug("Activity\n");

            } else {
                debug("Inactivity\n");
            }
        }
    }
}

int main() {
    if (!accel.init()) {
        debug("Could not init\n");
        return 0;
    }

    if (!accel.setupAccel(LSM6DS3::AccelODR_52Hz, LSM6DS3::AccelScale_4G)) {
        debug("Couldn't setup setup slow ODR accel\n");
        return 0;
    }

    if (!accel.setAccelFilter(LSM6DS3::AccelSlopeFilter)) {
        debug("Couldn't setup filter\n");
        return 0;
    }

    if (!accel.setWakeup(2, 2, 1)) { // threshold, wakeup time, sleep time
        debug("Couldn't setup wakeup\n");
        return 0;
    }

    if (!accel.enableInactivity(true)) {
        debug("Couldn't setup inactivity\n");
        return 0;
    }

    if (!accel.setFnINT1(0b10000000)) {
        debug("Couldn't setup setup INT\n");
        return 0;
    }

    accelInt.rise(eQueue.event(accelIntCb));

    while (1) {
        eQueue.dispatch_forever();
    }
}
```

</details>

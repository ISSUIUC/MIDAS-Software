#include "sensors.h"

// #include sensor library
#include <Arduino_LSM6DS3.h>
// global static instance of the sensor
LSM6DS3Class LSM(SPI, LSM6DS3_CS, 46);

ErrorCode LowGLSMSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!LSM.begin()) {
        return ErrorCode::GyroCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

LowGLSM LowGLSMSensor::read() {
    // read from aforementioned global instance of sensor
    LowGLSM result;
    LSM.readAcceleration(result.ax, result.ay, result.az);
    LSM.readGyroscope(result.gx, result.gy, result.gz);
    return result;
}

#include "sensors.h"

// #include sensor library
#include <SparkFunLSM6DS3.h>
// global static instance of the sensor
LSM6DS3 LSM(SPI_MODE, LSM6DS3_CS);

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
    result.gx = LSM.readFloatGyroX();
    result.gy = LSM.readFloatGyroY(),
    result.gz = LSM.readFloatGyroZ();
    result.ax = LSM.readFloatAccelX();
    result.ay = LSM.readFloatAccelY();
    result.az = LSM.readFloatAccelZ();
    return result;
}
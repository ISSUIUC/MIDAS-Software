#include "sensors.h"

// #include sensor library
#include <SparkFunLSM6DS3.h>
// global static instance of the sensor
LSM6DS3 LSM(SPI_MODE, LSM6DS3_CS);

ErrorCode GyroscopeSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!LSM.begin()) {
        return ErrorCode::GyroCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

Gyroscope GyroscopeSensor::read() {
    // read from aforementioned global instance of sensor
    Gyroscope result;
    result.gx = LSM.readFloatGyroX();
    result.gy = LSM.readFloatGyroY(),
    result.gz = LSM.readFloatGyroZ();
    return result;
}
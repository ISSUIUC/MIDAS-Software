/**
 * @file LowGLSM.cpp
 * 
 * @brief Holds the function definitions for the LSM accelerometer + gyro
*/

#include "sensors.h"
#include <Arduino_LSM6DS3.h>

LSM6DS3Class LSM(SPI, LSM6DS3_CS, 46);      // global static instance of the sensor

/**
 * @brief Initializes the low G LSM sensor
 * 
 * @return Error Code
*/
ErrorCode LowGLSMSensor::init() {
    if (!LSM.begin()) {
        return ErrorCode::GyroCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

/**
 * @brief Reads and returns the data from the sensor
 * 
 * @return a LowGLSM packet with current acceleration and gyro in all three axes
*/
LowGLSM LowGLSMSensor::read() {
    // read from aforementioned global instance of sensor
    LowGLSM result;
    LSM.readAcceleration(result.ax, result.ay, result.az);
    LSM.readGyroscope(result.gx, result.gy, result.gz);
    return result;
}

#include "sensors.h"
#include <Adafruit_LIS3MDL.h>
#include "hal.h"
// #include sensor library

// global static instance of the sensor
Adafruit_LIS3MDL MagnetometerSensor;

ErrorCode Magnetometer::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!MagnetometerSensor.begin_SPI(LIS3MDL_CS)) {
        return ErrorCode::CannotConnectMagnetometer;
    }
    MagnetometerSensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    MagnetometerSensor.setDataRate(LIS3MDL_DATARATE_155_HZ);
    MagnetometerSensor.setRange(LIS3MDL_RANGE_4_GAUSS);
    return ErrorCode::NoError;
}

MagnetometerReading Magnetometer::read() {
    // read from aforementioned global instance of sensor
    MagnetometerSensor.read();

    float mx = MagnetometerSensor.x_gauss;
    float my = MagnetometerSensor.y_gauss;
    float mz = MagnetometerSensor.z_gauss;
    MagnetometerReading reading {mx, my, mz};
    return reading;
}
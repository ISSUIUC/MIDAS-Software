#include "sensors.h"
#include <Adafruit_LIS3MDL.h>
#include "hal.h"

// global static instance of the sensor
Adafruit_LIS3MDL LIS3MDL;

ErrorCode MagnetometerSensor::init() {
    if (!LIS3MDL.begin_SPI(LIS3MDL_CS)) { //Checks if sensor is connected
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    LIS3MDL.setOperationMode(LIS3MDL_CONTINUOUSMODE);//reading continously
    LIS3MDL.setDataRate(LIS3MDL_DATARATE_155_HZ);//sets datarate to 155hz
    LIS3MDL.setRange(LIS3MDL_RANGE_4_GAUSS);//earth is 1/2 gauss, can detect high current
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    LIS3MDL.read();

    float mx = LIS3MDL.x_gauss;
    float my = LIS3MDL.y_gauss;
    float mz = LIS3MDL.z_gauss;
    Magnetometer reading {mx, my, mz};
    return reading;
}
#include "sensors.h"
#include <Adafruit_LIS3MDL.h>
#include "hal.h"
// #include sensor library

// global static instance of the sensor
Adafruit_LIS3MDL MagnetometerSensor;

ErrorCode Magnetometer::init() {
    if (!MagnetometerSensor.begin_SPI(LIS3MDL_CS)) { //Checks if sensor is connected
        return ErrorCode::CannotConnectMagnetometer;
    }
    MagnetometerSensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);//reading continously
    MagnetometerSensor.setDataRate(LIS3MDL_DATARATE_155_HZ);//sets datarate to 155hz
    MagnetometerSensor.setRange(LIS3MDL_RANGE_4_GAUSS);//earth is 1/2 gauss, can detect high current
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
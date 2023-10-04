#include "sensors.h"
#include "lib/Adxl355.h"

// #include sensor library

// global static instance of the sensor

#define cs_pin 10

Adxl355 sensor;

void LowGSensor::calibrate()
{
    sensor.calibrateSensor(cs_pin);
}

ErrorCode LowGSensor::init()
{
    ErrorCode error = ErrorCode::NoError;
    sensor.initSPI(SPI);
    sensor.start();
    delay(1000);

    if (sensor.isDeviceRecognized()) {

    } else {
        error = ErrorCode::SensorNotRecognized;
    }

    // Defaults to 2G range and Output Data Rate: 4000Hz and Low Pass Filter: 1000Hz
    sensor.initializeSensor();

    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return error;
}

LowGData LowGSensor::read()
{
    // read from aforementioned global instance of sensor
    return LowGData();
}
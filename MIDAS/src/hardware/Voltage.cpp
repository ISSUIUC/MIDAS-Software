#include "sensors.h"

// #include sensor library

// global static instance of the sensor


ErrorCode VoltageSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Voltage VoltageSensor::read() {
    // read from aforementioned global instance of sensor
    return Voltage();
}
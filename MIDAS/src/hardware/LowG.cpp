#include "sensors.h"

// #include sensor library

// global static instance of the sensor


ErrorCode LowGSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

LowGData LowGSensor::read() {
    // read from aforementioned global instance of sensor
    return LowGData();
}

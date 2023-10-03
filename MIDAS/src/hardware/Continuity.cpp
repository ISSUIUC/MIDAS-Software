#include "sensors.h"

// #include sensor library

// global static instance of the sensor


ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    return Continuity();
}
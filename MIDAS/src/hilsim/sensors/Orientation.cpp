#include "sensors.h"
#include "../kamaji/kal_rocket.h"
// #include sensor library

// global static instance of the sensor


ErrorCode OrientationSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Orientation OrientationSensor::read() {
    // read from aforementioned global instance of sensor
    return GLOBAL_DATA.orientation;
}

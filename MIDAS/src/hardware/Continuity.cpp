#include "sensors.h"
#include <hal.h>

// #include sensor library

// global static instance of the sensor


ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    pinMode(input_pin, INPUT);
    pinMode(output_pin, OUTPUT);
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    return Continuity(digitalRead(input_pin));
}

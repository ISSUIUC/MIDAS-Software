#include "sensors.h"
#include "drivers/ads7138-q1.h"

#include <hal.h>

ADS7138_1Q ads7138_1q;

ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    //pinMode(input_pin, INPUT);
    //pinMode(output_pin, OUTPUT);
    // Ask ADS to init the pins, we still need to get the device to actually read
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    Continuity continuity;
    for (int i = 0; i < PIN_COUNT; i++) {
        continuity[i] = ads7138_1q.read_pin(i);
    }
    return continuity;
}

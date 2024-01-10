#include "sensors.h"
#include "ads7138-q1.h"

#include <hal.h>

ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!ADS7138Init()) {
        return ErrorCode::ContinuityCouldNotBeInitialized;
    }
    // Ask ADS to init the pins, we still need to get the device to actually read
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    Continuity continuity;
    for (int i = 0; i < CONTINUITY_PIN_COUNT; i++) {
        continuity.pins[i] = adcAnalogRead(ADCAddress{i}).value;
    }
    return continuity;
}

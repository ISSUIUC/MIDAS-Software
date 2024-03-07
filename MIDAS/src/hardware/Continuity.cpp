#include "sensors.h"
#include "ads7138-q1.h"

#include <hal.h>

ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    ADS7138Init();
//    if (!) {
//        return ErrorCode::ContinuityCouldNotBeInitialized;
//    }
    // Ask ADS to init the pins, we still need to get the device to actually read
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    Continuity continuity;
    // todo convert to bool
    continuity.sense_pyro = adcAnalogRead(ADCAddress{SENSE_PYRO}).value * 3.3f / 4096.f;
    continuity.pins[0] = adcAnalogRead(ADCAddress{SENSE_MOTOR}).value * 3.3f / 4096.f;
    continuity.pins[1] = adcAnalogRead(ADCAddress{SENSE_MAIN}).value * 3.3f / 4096.f;
    continuity.pins[2] = adcAnalogRead(ADCAddress{SENSE_APOGEE}).value * 3.3f / 4096.f;
    continuity.pins[3] = adcAnalogRead(ADCAddress{SENSE_AUX}).value * 3.3f / 4096.f;
    return continuity;
}

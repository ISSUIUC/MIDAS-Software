#include "sensors.h"
#include "ads7138-q1.h"

#include <hal.h>

#define PYRO_VOLTAGE_DIVIDER (100.0 / (100.0 + 220.0 ))
#define CONT_VOLTAGE_DIVIDER (10.0 / (10.0 + 22.0))

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
    continuity.sense_pyro = adcAnalogRead(ADCAddress{SENSE_PYRO}).value * 3.3f / 4096.f / PYRO_VOLTAGE_DIVIDER;
    continuity.pins[0] = adcAnalogRead(ADCAddress{SENSE_MOTOR}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[1] = adcAnalogRead(ADCAddress{SENSE_MAIN}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[2] = adcAnalogRead(ADCAddress{SENSE_APOGEE}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[3] = adcAnalogRead(ADCAddress{SENSE_AUX}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    return continuity;
}

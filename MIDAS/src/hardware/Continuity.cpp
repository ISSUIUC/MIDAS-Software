#include "sensors.h"
#include "ads7138-q1.h"

#include "pins.h"

#define PYRO_VOLTAGE_DIVIDER (5.0f / (5.0f + 20.0f))       //voltage divider for pyro batt voltage, check hardware schematic
#define CONT_VOLTAGE_DIVIDER (5.0f / (5.0f + 20.0f))       //voltage divider for continuity voltage, check hardware schematic

/**
 * @brief Initializes ADC, returns NoError
 * 
 * @return Error code
*/
ErrorCode ContinuitySensor::init() {
    ADS7138Init();              // Ask ADS to init the pins, we still need to get the device to actually read

    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the ADC
 * 
 * @return Continuity data packet
*/
ContinuityData ContinuitySensor::read() {
    ContinuityData continuity {
        .sense_pyro = static_cast<float>(adcAnalogRead(ADCAddress{SENSE_PYRO}).value) * 3.3f / 4096.f / PYRO_VOLTAGE_DIVIDER,
        .pins = {
            static_cast<float>(adcAnalogRead(ADCAddress{SENSE_MOTOR}).value) * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER,
            static_cast<float>(adcAnalogRead(ADCAddress{SENSE_MAIN}).value) * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER,
            static_cast<float>(adcAnalogRead(ADCAddress{SENSE_APOGEE}).value) * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER,
            static_cast<float>(adcAnalogRead(ADCAddress{SENSE_AUX}).value) * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER,
        }
    };
    return continuity;
}

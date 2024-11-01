#include "sensors.h"
#include "ads7138-q1.h"
#include <hal.h>
#include <queue>


#define PYRO_VOLTAGE_DIVIDER (5.0 / (5.0 + 20.0))       //voltage divider for pyro batt voltage, check hardware schematic
#define CONT_VOLTAGE_DIVIDER (5.0 / (5.0 + 20.0))       //voltage divider for continuity voltage, check hardware schematic
extern std::queue<std::string> logQueue; 

/**
 * @brief Initializes ADC, returns NoError
 * 
 * @return Error code
*/
ErrorCode ContinuitySensor::init() {
    ADS7138Init();              // Ask ADS to init the pins, we still need to get the device to actually read
    logQueue.push("BarometerInitialized");//process profiling
    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the ADC
 * 
 * @return Continuity data packet
*/
Continuity ContinuitySensor::read() {
    Continuity continuity;
    //ADC reference voltage is 3.3, returns 12 bit value
    continuity.sense_pyro = adcAnalogRead(ADCAddress{SENSE_PYRO}).value * 3.3f / 4096.f / PYRO_VOLTAGE_DIVIDER;
    continuity.pins[0] = adcAnalogRead(ADCAddress{SENSE_MOTOR}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[1] = adcAnalogRead(ADCAddress{SENSE_MAIN}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[2] = adcAnalogRead(ADCAddress{SENSE_APOGEE}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    continuity.pins[3] = adcAnalogRead(ADCAddress{SENSE_AUX}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    return continuity;
}

#include "sensors.h"
#include <hal.h>
#include <Wire.h>

int read_reg(int reg, int bytes) {
    Wire.beginTransmission(0x44);
    Wire.write(reg);
    if(Wire.endTransmission()){
       return -1;
    }
    Wire.requestFrom(0x40, bytes);
    int val = 0;
    for(int i = 0; i < bytes; i++){
        int v = Wire.read();
        if(v == -1) return val;
        val = (val << 8) | v;
    }
    return val;
}


/**
 * @brief Initializes ADC, returns NoError
 * 
 * @return Error code
*/
ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the ADC
 * 
 * @return Continuity data packet
*/
Continuity ContinuitySensor::read() {
    Continuity continuity;
    // get battery voltage + current
    // based on those and resistor values, we can determine what 
    // pyros are continious or not
    int current = read_reg(0x7, 2);
    int voltage = read_reg(0x5, 2);

    float true_voltage = voltage * 3.125 / 1000.0;
    float true_current = current * 1.2 / 1000.0;
    
    // now do a bunch of logic to fill this in

    // continuity.sense_pyro = adcAnalogRead(ADCAddress{SENSE_PYRO}).value * 3.3f / 4096.f / PYRO_VOLTAGE_DIVIDER;
    // continuity.pins[0] = adcAnalogRead(ADCAddress{SENSE_MOTOR}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    // continuity.pins[1] = adcAnalogRead(ADCAddress{SENSE_MAIN}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    // continuity.pins[2] = adcAnalogRead(ADCAddress{SENSE_APOGEE}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;
    // continuity.pins[3] = adcAnalogRead(ADCAddress{SENSE_AUX}).value * 3.3f / 4096.f / CONT_VOLTAGE_DIVIDER;

    return continuity;
}

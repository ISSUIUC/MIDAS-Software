#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>
#include <Wire.h>

#define ADC_I2C_ADDR 0x14

ADS7138 ADC;

/**
 * @brief "Initializes" the voltage sensor. Since it reads directly from a pin without a library, there is no specific initialization.
 * 
 * @return Error Code, will always be NoError
*/
ErrorCode VoltageSensor::init() {
    if(!ADC.init(&Wire, ADC_I2C_ADDR)) {
        return ErrorCode::ADCFailedToInit;
    }
    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the given analog pin and converts it to a battery voltage with the assumption that the voltage sensor is plugged into that pin
 * 
 * @return The scaled voltage given by the voltage sensor
*/
Voltage VoltageSensor::read() {
    Voltage voltage;

    voltage.v_Bat = ADC.read(VBAT_SENSE);
    voltage.v_Pyro = ADC.read(PYRO_SENSE);
    voltage.continuity[0] = ADC.read(SENSE_A);
    voltage.continuity[1] = ADC.read(SENSE_B);
    voltage.continuity[2] = ADC.read(SENSE_C);
    voltage.continuity[3] = ADC.read(SENSE_D);

    return voltage;
}
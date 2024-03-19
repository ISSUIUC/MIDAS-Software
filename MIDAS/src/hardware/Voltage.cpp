#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>

/**
 * "Initializes" the voltage sensor. Since it reads directly from a pin without a library, there is no specific initialization.
*/
ErrorCode VoltageSensor::init() {
    return ErrorCode::NoError;
}

/**
 * Reads the value of the given analog pin and converts it to a battery voltage with the assumption that the voltage sensor is plugged into that pin
 * \return The scaled voltage given by the voltage sensor
*/
Voltage VoltageSensor::read() {
    Voltage v_battery;
    v_battery.voltage = adcAnalogRead(ADCAddress{VOLTAGE_PIN}).value * 3.3f / 4096.f;
    return v_battery;
}

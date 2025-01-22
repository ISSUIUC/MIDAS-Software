#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>

#include "pins.h"

#define VOLTAGE_DIVIDER (5.0f / (5.0f + 20.0f))

/**
 * @brief "Initializes" the voltage sensor. Since it reads directly from a pin without a library, there is no specific initialization.
 * 
 * @return Error Code, will always be NoError
*/
ErrorCode VoltageSensor::init() {
    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the given analog pin and converts it to a battery voltage with the assumption that the voltage sensor is plugged into that pin
 * 
 * @return The scaled voltage given by the voltage sensor
*/
VoltageData VoltageSensor::read() {
    VoltageData v_battery;
    v_battery.voltage = static_cast<float>(adcAnalogRead(ADCAddress{VOLTAGE_PIN}).value) * 3.3f / 4095.0f / VOLTAGE_DIVIDER;
//    Serial.print("Raw voltage reading: ");
//    Serial.print(v_battery.voltage);
//    Serial.println("");
    //* 3.3f / 4095.f / VOLTAGE_DIVIDER;
    return v_battery;
}

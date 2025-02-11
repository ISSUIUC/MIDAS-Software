#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>
#include <Wire.h>

#define VOLTAGE_DIVIDER (5.0 / (5.0 + 20.0))

int read_board_pwr_monitor_register(int reg, int bytes) {
    Wire1.beginTransmission(0x44);
    Wire1.write(reg);
    if(Wire1.endTransmission()){
        Serial.println("I2C Error");
    }

    Wire1.requestFrom(0x44, bytes);
    int val = 0;

    for(int i = 0; i < bytes; i++){
        int v = Wire1.read();
        if(v == -1) Serial.println("I2C Read Error");
        val = (val << 8) | v;
    }

    return val;
}

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
Voltage VoltageSensor::read() {
    Voltage v_battery;
    int voltage = read_board_pwr_monitor_register(0x5, 2);
    int16_t current = read_board_pwr_monitor_register(0x7, 2);

    float voltage_normalized = voltage * 3.125 / 1000.0;
    float absolute_current = current * 1.2 / 1000.0;

    // Serial.print("Voltage: ");
    // Serial.println(voltage_normalized);
    // Serial.print("Current: ");
    // Serial.println(current);

    v_battery.voltage = voltage_normalized;
    v_battery.current = absolute_current;
//    Serial.print("Raw voltage reading: ");
//    Serial.print(v_battery.voltage);
//    Serial.println("");
    //* 3.3f / 4095.f / VOLTAGE_DIVIDER;
    return v_battery;
}

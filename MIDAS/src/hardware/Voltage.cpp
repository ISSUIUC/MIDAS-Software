#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>

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
    int voltage = read_reg(0x5, 2);

    v_battery.voltage = voltage * 3.125 / 1000.0;

    return v_battery;
}

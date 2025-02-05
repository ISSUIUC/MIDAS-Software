#include "sensors.h"
#include "ads7138-q1.h"
#include <hal.h>
#include <Wire.h>

#define PYRO_VOLTAGE_DIVIDER (5.0 / (5.0 + 20.0))       //voltage divider for pyro batt voltage, check hardware schematic
#define CONT_VOLTAGE_DIVIDER (5.0 / (5.0 + 20.0))       //voltage divider for continuity voltage, check hardware schematic

// Reads the given register from the pyro power monitor
int read_pwr_monitor_register(int reg, int bytes) {
    Wire1.beginTransmission(0x41); // I2C Address 0x41 is pyro pwr monitor
    Wire1.write(reg);
    if(Wire1.endTransmission()){
        Serial.println("I2C Error");
    }

    Wire1.requestFrom(0x41, bytes);
    int val = 0;

    for(int i = 0; i < bytes; i++){
        int v = Wire1.read();
        if(v == -1) Serial.println("I2C Read Error");
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
    // ADS7138Init();              // Ask ADS to init the pins, we still need to get the device to actually read
    // Configure the INA745b        MODE:ContTCV  VBUS:1052us  VSEN:1052us  TCT:1052us   AVG:128
    constexpr uint16_t INA_config = (0xF << 12) | (0x5 << 9) | (0x5 << 6) | (0x5 << 3) | (0x4);

    Wire1.beginTransmission(0x41);
    Wire1.write(0x1);

    Wire1.write(((INA_config >> 8) & 0xFF));
    Wire1.write(((INA_config >> 0) & 0xFF));
    
    if(Wire1.endTransmission()){
        Serial.println("Pyro PWR I2C Error");
        return ErrorCode::ContinuityCouldNotBeInitialized;
    }

    Serial.println("Pyro PWR monitor configured");

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

    // MIDAS 2.1 rev A ADC sense fix:
    int16_t current = read_pwr_monitor_register(0x7, 2);
    int voltage = read_pwr_monitor_register(0x5, 2);

    float voltage_normalized = voltage * 3.125 / 1000.0; // V

    float continuous_channels = 0.0;
    float absolute_current = 0.0;
    float expected_current = 0.0;

    if(voltage_normalized > 1) {
        absolute_current = current * 1.2 / 1000.0;

        expected_current = (voltage_normalized - 0.2) / 470.0; // Account for diode voltage drop
        continuous_channels = absolute_current / expected_current;
    }

    // We don't have the granularity to determine individual voltages, so all of them will give the # of continuous channels
    continuity.pins[0] = continuous_channels; // Number of continuous channels on the pyro bus
    continuity.pins[1] = absolute_current;    // The absolute current running through the pyro bus
    continuity.pins[2] = expected_current;    // Calculated expected current based on current pyro bus voltage
    continuity.pins[3] = voltage_normalized;  // Pyro bus voltage
    return continuity;
}

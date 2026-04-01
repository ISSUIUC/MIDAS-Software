#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>
#include <Wire.h>

constexpr float VOLTAGE_DIVIDER = (5.0 / (5.0 + 20.0));
constexpr float PYRO_VOLTAGE_DIVIDER = (18.0/(100.0+18.0));
constexpr float PYRO_BATT_VOLTAGE_DIVIDER = (100.0/(100.0+560.0));
constexpr float BATT_VOLTAGE_DIVIDER = (100.0/(100.0+100.0));
constexpr float VOLTAGE_SCALE = 3.3;
constexpr int VOLTAGE_REG_WIDTH = (1<<16);

constexpr ADCAddress pinA = {SENSE_A};
constexpr ADCAddress pinB = {SENSE_B};
constexpr ADCAddress pinC = {SENSE_C};
constexpr ADCAddress pinD = {SENSE_D};
constexpr ADCAddress pinBat = {VBAT_SENSE};
constexpr ADCAddress pinPyro = {PYRO_SENSE};

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
    Voltage voltage;

    //Getting ADC values
    AdcReadResult sensorA = adcAnalogRead(pinA);
    AdcReadResult sensorB = adcAnalogRead(pinB);
    AdcReadResult sensorC = adcAnalogRead(pinC);
    AdcReadResult sensorD = adcAnalogRead(pinD);
    AdcReadResult sensorBat = adcAnalogRead(pinBat);
    AdcReadResult sensorPyro = adcAnalogRead(pinPyro);
    
    //Set output to pin A to avoid pyro battery voltage leaking into next reading
    adcSetOutput(pinA); 

    //Converts ADC value to voltage for each pin
    if(sensorBat.error == AdcError::NoError){
        float Bat_voltage = ((static_cast<float>(sensorBat.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE) / BATT_VOLTAGE_DIVIDER; 
        voltage.v_Bat = Bat_voltage;
    }

    if(sensorPyro.error == AdcError::NoError){
        float Pyro_voltage = ((static_cast<float>(sensorPyro.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE)/(PYRO_BATT_VOLTAGE_DIVIDER); //Accounting for voltage divider on MIDAS mini
        voltage.v_Pyro = Pyro_voltage;
    }

    if(sensorA.error == AdcError::NoError){
        float A_voltage = ((static_cast<float>(sensorA.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE) / PYRO_VOLTAGE_DIVIDER;
        voltage.continuity[0] = A_voltage;
    }

    if(sensorB.error == AdcError::NoError){
        float B_voltage = ((static_cast<float>(sensorB.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE) / PYRO_VOLTAGE_DIVIDER;
        voltage.continuity[1] = B_voltage;
    }

    if(sensorC.error == AdcError::NoError){
        float C_voltage = ((static_cast<float>(sensorC.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE) / PYRO_VOLTAGE_DIVIDER;
        voltage.continuity[2] = C_voltage;
    }

    if(sensorD.error == AdcError::NoError){
        float D_voltage = ((static_cast<float>(sensorD.value) / VOLTAGE_REG_WIDTH) * VOLTAGE_SCALE) / PYRO_VOLTAGE_DIVIDER;
        voltage.continuity[3] = D_voltage;
    }

    return voltage;
}
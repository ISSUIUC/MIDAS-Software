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
    Voltage voltage;

    //CHECK PINS
    ADCAddress pinA;
    ADCAddress pinB;
    ADCAddress pinC;
    ADCAddress pinD;
    ADCAddress pinBat;
    ADCAddress pinPyro;
    pinA.pin_id = 0;
    pinB.pin_id = 1;
    pinC.pin_id = 4;
    pinD.pin_id = 5; 
    pinBat.pin_id = 7; 
    pinPyro.pin_id = 2; 

    //Getting ADC values
    AdcReadResult sensorA = adcAnalogRead(pinA);
    AdcReadResult sensorB = adcAnalogRead(pinB);
    AdcReadResult sensorC = adcAnalogRead(pinC);
    AdcReadResult sensorD = adcAnalogRead(pinD);
    AdcReadResult sensorBat = adcAnalogRead(pinBat);
    AdcReadResult sensorPyro = adcAnalogRead(pinPyro);

    //Converts ADC value to voltage for each pin
    if(sensorBat.error == AdcError::NoError){
        float Bat_value = sensorBat.value;
        float Bat_voltage = (((float)Bat_value / 65535) * 3.3) * 2.0; 
        voltage.v_Bat = Bat_voltage;
        Serial.print("VBAT: ");
        Serial.println(Bat_voltage); 
    }

    if(sensorPyro.error == AdcError::NoError){
        float Pyro_voltage = (((float)sensorPyro.value / 65535) * 3.3)/(100.0/660); //Accounting for voltage divider on MIDAS mini
        voltage.v_Pyro = Pyro_voltage;
        Serial.print("PYRO: ");
        Serial.println(Pyro_voltage);
    }

    if(sensorA.error == AdcError::NoError){
        float A_value = sensorA.value* (100000.0/118000);                         //accounting for the resisters
        float A_voltage = (((float)A_value / 65535) * 3.3)/(18.0/100);
        voltage.continuity[0] = A_voltage;
        Serial.print("A: ");
        Serial.println(A_voltage);
    }

    if(sensorB.error == AdcError::NoError){
        uint16_t B_value = sensorB.value* (100000.0/118000);                       //accounting for the resisters
        float B_voltage = (((float)B_value / 65535) * 3.3)/(18.0/100);
        voltage.continuity[1] = B_voltage;
        Serial.print("B: ");
        Serial.println(B_voltage);
    }

    if(sensorC.error == AdcError::NoError){
        uint16_t C_value = sensorC.value*(100000.0/118000);                      //accounting for the resisters
        float C_voltage = (((float)C_value / 65535) * 3.3)/(18.0/100);
        voltage.continuity[2] = C_voltage;
        Serial.print("C: ");
        Serial.println(C_voltage);
    }

    if(sensorD.error == AdcError::NoError){
        uint16_t D_value = sensorD.value*(100000.0/118000);                     //accounting for the resisters
        float D_voltage = (((float)D_value / 65535) * 3.3)/(18.0/100);
        voltage.continuity[3] = D_voltage;
        Serial.print("D: ");
        Serial.println(D_voltage);
    }

    return voltage;
}

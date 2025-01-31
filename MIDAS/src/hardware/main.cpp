// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------
#include<Arduino.h>
#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setup() {
    WIRE.begin(6,5);

    Serial.begin(9600);
}

int read_reg(int reg, int bytes) {
    Wire.beginTransmission(0x40);
    Wire.write(reg);
    if(Wire.endTransmission()){
        Serial.println("I2C Error");
    }

    Wire.requestFrom(0x40, bytes);
    int val = 0;
    for(int i = 0; i < bytes; i++){
        int v = Wire.read();
        if(v == -1) Serial.println("I2C Read Error");
        val = (val << 8) | v;
    }
    return val;
}

void loop() {
    int power = read_reg(0x8, 3);
    int current = read_reg(0x7, 2);
    int temp = read_reg(0x6, 2);
    int voltage = read_reg(0x5, 2);
    Serial.print("Voltage ");
    Serial.println(voltage * 3.125 / 1000.0);
    Serial.print("Temp ");
    Serial.println(temp * 125 / 1000.0);
    Serial.print("Current ");
    Serial.println(current * 1.2 / 1000.0);
    Serial.print("Power ");
    Serial.println(power * 240 / 1000000.0);
    delay(1000);
}

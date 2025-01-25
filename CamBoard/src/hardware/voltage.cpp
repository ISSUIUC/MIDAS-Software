#include <Wire.h>
#include <HardwareSerial.h>

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
#include "ads7138-q1.h"
#include<Arduino.h>
#include<Wire.h>

#define WIRE Wire

#define ADC_ADDR 0x14
#define ADC_REG_READ 0x10
#define ADC_REG_WRITE 0x08
#define ADC_SET_BIT 0x18
#define ADC_CLEAR_BIT 0x20
#define ADC_REG_READ_CONTINUOUS 0x30
#define ADC_REG_WRITE_CONTINUOUS 0x28
#define ADC_SYSTEM_STATUS 0x00
#define ADC_MANUAL_CH_SEL 0x11

static int adc_reg_read(uint8_t address, uint8_t reg_address) {
  WIRE.beginTransmission(address);
  WIRE.write(ADC_REG_READ);
  WIRE.write(reg_address);
  if(!WIRE.endTransmission(true)){
    Serial.println("err1");
  }
  uint8_t ct = WIRE.requestFrom(address, 1, 1);
  if(ct != 1){
    Serial.println("err2");
  }
  return WIRE.read();
}

static void adc_reg_write(uint8_t address, uint8_t reg_address, uint8_t data) {
  WIRE.beginTransmission(address);
  WIRE.write(ADC_REG_WRITE);
  WIRE.write(reg_address);
  WIRE.write(data);
  if(!WIRE.endTransmission(true)){
    Serial.println("err3");
  }
}

uint16_t analogRead(ADCAddress pin){
    if(pin.pin_id < 0 || pin.pin_id >= 8){
        Serial.println("Err bad pin id");
        return 0;
    }

    adc_reg_write(ADC_ADDR, ADC_MANUAL_CH_SEL, pin.pin_id);
    
    int ct = WIRE.requestFrom(ADC_ADDR, 2, 1);
    if(ct != 2){
        Serial.println("Err read error");
    }
    int high = WIRE.read();
    int low = WIRE.read();
    return (high << 4) + (low >> 4);
}

bool ADS7138Init(){
  return adc_reg_read(ADC_ADDR, ADC_SYSTEM_STATUS) != 0;
}

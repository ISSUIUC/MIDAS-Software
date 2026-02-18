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

struct AdcRegReadResult {
  int value;
  AdcError error;
};

static AdcRegReadResult adc_reg_read(uint8_t address, uint8_t reg_address) {
  WIRE.beginTransmission(address);
  WIRE.write(ADC_REG_READ);
  WIRE.write(reg_address);
  if(WIRE.endTransmission(true) != 0){
    return AdcRegReadResult{.value=0, .error=AdcError::I2CError};
  }
  uint8_t ct = WIRE.requestFrom((int)address, 1, 1);
  if(ct != 1){
    return AdcRegReadResult{.value=0, .error=AdcError::I2CError};
  }
  int value = WIRE.read();
  return AdcRegReadResult{.value=value, .error=AdcError::NoError};
}

static AdcError adc_reg_write(uint8_t address, uint8_t reg_address, uint8_t data) {
  WIRE.beginTransmission(address);
  WIRE.write(ADC_REG_WRITE);
  WIRE.write(reg_address);
  WIRE.write(data);
  if(WIRE.endTransmission(true) != 0){
    return AdcError::I2CError;
  }
  return AdcError::NoError;
}

bool adcSetOutput(ADCAddress pin){
    if(pin.pin_id < 0 || pin.pin_id >= 8){
        return false;
    }

    adc_reg_write(ADC_ADDR, ADC_MANUAL_CH_SEL, pin.pin_id);

    return true;
}

AdcReadResult adcAnalogRead(ADCAddress pin){ 
    if(!adcSetOutput(pin)){
        return AdcReadResult{.value=0, .error=AdcError::InvalidPinError};
    }

    int ct = WIRE.requestFrom((int)ADC_ADDR, 2, 1);
    if(ct != 2){
        return AdcReadResult{.value=0, .error=AdcError::I2CError};
    }
    int high = WIRE.read();
    int low = WIRE.read();
    uint16_t value = (high << 8) + low;
    return AdcReadResult{.value=value, .error=AdcError::NoError};
}

bool ADS7138Init(){
  AdcRegReadResult reg = adc_reg_read(ADC_ADDR, ADC_SYSTEM_STATUS);
  if(reg.error != AdcError::NoError){
    return false;
  }
  return reg.value != 0;
}
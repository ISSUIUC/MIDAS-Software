#include"TCAL9539.h"
#include<Wire.h>

static constexpr uint8_t REG_INPUT0 = 0x0;
static constexpr uint8_t REG_INPUT1 = 0x1;
static constexpr uint8_t REG_OUTPUT0 = 0x2;
static constexpr uint8_t REG_OUTPUT1 = 0x3;
static constexpr uint8_t POLARITY_INVERSION0 = 0x4;
static constexpr uint8_t POLARITY_INVERSION1 = 0x5;
static constexpr uint8_t REG_CONFIG0 = 0x6;
static constexpr uint8_t REG_CONFIG1 = 0x7;

bool TCAL9539Init(int reset_pin){
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
    delay(1);
    digitalWrite(reset_pin, LOW);
    delay(1);
    digitalWrite(reset_pin, HIGH);

    uint8_t addrs[] = {GPIO0_ADDRESS, GPIO1_ADDRESS, GPIO2_ADDRESS};
    
    TwoWire* wirezero = &Wire;
    TwoWire* wireone = &Wire1;

    for(uint8_t addr : addrs){
        Serial.print("Testing ");
        Serial.println(addr);

        //wirezero
        wirezero->beginTransmission(addr);
        wirezero->write(REG_OUTPUT0);
        if(!wirezero->endTransmission()){
            Serial.println("Failed at endTransmission");
            return false;
        }
        int ct = wirezero->requestFrom(addr, 1);
        if(ct != 1){
            Serial.println("Failed at requestFrom");
            return false;
        }
        int v = wirezero->read();
        //REG_OUTPUT0 is set all ones on power up
        if(v != 0xff){
            Serial.println("Failed at REG_OUTPUT0");
            return false;
        }

        //wireone
        wireone->beginTransmission(addr);
        wireone->write(REG_OUTPUT0);
        if(!wireone->endTransmission()){
            Serial.println("Failed at endTransmission");
            return false;
        }
        ct = wireone->requestFrom(addr, 1);
        if(ct != 1){
            Serial.println("Failed at requestFrom");
            return false;
        }
        v = wireone->read();
        //REG_OUTPUT0 is set all ones on power up
        if(v != 0xff){
            Serial.println("Failed at REG_OUTPUT0");
            return false;
        }
    }
    return true;
}

static uint8_t pin_state[2][3][2] = {{{0xff,0xff},{0xff,0xff},{0xff,0xff}}, {{0xff,0xff},{0xff,0xff},{0xff,0xff}}};
static uint8_t pin_config[2][3][2] = {{{0xff,0xff},{0xff,0xff},{0xff,0xff}}, {{0xff,0xff},{0xff,0xff},{0xff,0xff}}};

GpioError gpioDigitalWrite(GpioAddress addr, int mode, int whichwire){
    TwoWire* wire;
    if (whichwire == 0) {
        wire = &Wire;
    } else {
        wire = &Wire1;
    }

    if(!addr.is_valid) {
        return GpioError::InvalidPinError;
    }

    uint8_t current_state = pin_state[whichwire][addr.gpio_id][addr.port_idx];
    if(mode == HIGH){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == LOW){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }

    wire->beginTransmission(addr.gpio_address);
    wire->write(REG_OUTPUT0 + addr.port_idx);
    wire->write(current_state);
    pin_state[whichwire][addr.gpio_id][addr.port_idx] = current_state;
    int err = 0;
    err = wire->endTransmission(true);
    if(err != 0){
        return GpioError::I2CError;
    }
    return GpioError::NoError;
}

GpioReadResult gpioDigitalRead(GpioAddress addr, int whichwire){
    TwoWire* wire;
    if (whichwire == 0) {
        wire = &Wire;
    } else {
        wire = &Wire1;
    }

    if(!addr.is_valid) {
        return GpioReadResult{.value=LOW,.error=GpioError::InvalidPinError};
    }

    wire->beginTransmission(addr.gpio_address);
    wire->write(REG_INPUT0 + addr.port_idx);
    if(!wire->endTransmission(true)){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }
    int ct = wire->requestFrom(addr.gpio_address, 1);
    if(ct != 1){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }

    uint8_t val = wire->read();    
    return GpioReadResult{.value=(val & (1 << addr.pin_offset)) != 0,.error=GpioError::NoError};
}

GpioError gpioPinMode(GpioAddress addr, int mode, int whichwire){
    TwoWire* wire;
    if (whichwire == 0) {
        wire = &Wire;
    } else {
        wire = &Wire1;
    }

    if(!addr.is_valid){
        return GpioError::NoError;
    }

    uint8_t current_state = pin_config[whichwire][addr.gpio_id][addr.port_idx];

    if(mode == INPUT){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == OUTPUT){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }
    
    GpioError err = gpioDigitalWrite(addr, LOW, whichwire);
    if(err != GpioError::NoError){
        return err;
    }

    //wirezero
    wire->beginTransmission(addr.gpio_address);
    wire->write(REG_CONFIG0 + addr.port_idx);
    wire->write(current_state);
    pin_config[whichwire][addr.gpio_id][addr.port_idx] = current_state;

    if(wire->endTransmission(true) != 0){
        return GpioError::I2CError;
    }
    return GpioError::NoError;
}

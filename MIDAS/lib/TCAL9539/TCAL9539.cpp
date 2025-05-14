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

TwoWire& tcal_get_wire_by_id(int index) {
    if(I2C_BUS_Map[index] == 0) {
        return Wire; 
    } else {
        return Wire1;
    }
}

bool TCAL9539Init(int reset_pin){
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
    delay(1);
    digitalWrite(reset_pin, LOW);
    delay(1);
    digitalWrite(reset_pin, HIGH);

    uint8_t addrs[] = {GPIO0_ADDRESS, GPIO1_ADDRESS, GPIO2_ADDRESS};
    
    for(int i = 0; i < std::size(addrs); i++){
        TwoWire& wire = tcal_get_wire_by_id(i);
        uint8_t addr = addrs[i];

        wire.beginTransmission(addr);
        wire.write(REG_OUTPUT0);
        if(wire.endTransmission() != 0){
            Serial.print("Failed at endTransmission ");
            Serial.println(addr);
            return false;
        }
        int ct = wire.requestFrom(addr, 1);
        if(ct != 1){
            Serial.print("Failed at requestFrom ");
            Serial.println(addr);
            return false;
        }
        int v = wire.read();
        //REG_OUTPUT0 is set all ones on power up
        if(v != 0xff){
            Serial.println("Failed at REG_OUTPUT0");
            return false;
        }
    }
    return true;
}

static uint8_t pin_state[3][2] = {{0xff,0xff},{0xff,0xff},{0xff,0xff}};
static uint8_t pin_config[3][2] = {{0xff,0xff},{0xff,0xff},{0xff,0xff}};

GpioError gpioDigitalWrite(GpioAddress addr, int mode){


    if(!addr.is_valid) {
        return GpioError::InvalidPinError;
    }

    TwoWire& wire = tcal_get_wire_by_id(addr.gpio_id);

    uint8_t current_state = pin_state[addr.gpio_id][addr.port_idx];
    if(mode == HIGH){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == LOW){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }

    wire.beginTransmission(addr.gpio_address);
    wire.write(REG_OUTPUT0 + addr.port_idx);
    wire.write(current_state);
    pin_state[addr.gpio_id][addr.port_idx] = current_state;
    int err = 0;
    err = wire.endTransmission(true);
    if(err != 0){
        return GpioError::I2CError;
    }
    return GpioError::NoError;
}

GpioReadResult gpioDigitalRead(GpioAddress addr){
    if(!addr.is_valid) {
        return GpioReadResult{.value=LOW,.error=GpioError::InvalidPinError};
    }

    TwoWire& wire = tcal_get_wire_by_id(addr.gpio_id);

    wire.beginTransmission(addr.gpio_address);
    wire.write(REG_INPUT0 + addr.port_idx);
    if(wire.endTransmission(true) != 0){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }
    int ct = wire.requestFrom(addr.gpio_address, 1);
    if(ct != 1){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }

    uint8_t val = wire.read();    
    return GpioReadResult{.value=(val & (1 << addr.pin_offset)) != 0,.error=GpioError::NoError};
}

GpioError gpioPinMode(GpioAddress addr, int mode){
    if(!addr.is_valid){
        return GpioError::NoError;
    }

    TwoWire& wire = tcal_get_wire_by_id(addr.gpio_id);

    uint8_t current_state = pin_config[addr.gpio_id][addr.port_idx];

    if(mode == INPUT){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == OUTPUT){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }
    
    GpioError err = gpioDigitalWrite(addr, LOW);
    if(err != GpioError::NoError){
        return err;
    }

    //wirezero
    wire.beginTransmission(addr.gpio_address);
    wire.write(REG_CONFIG0 + addr.port_idx);
    wire.write(current_state);
    pin_config[addr.gpio_id][addr.port_idx] = current_state;

    if(wire.endTransmission(true) != 0){
        return GpioError::I2CError;
    }
    return GpioError::NoError;
}
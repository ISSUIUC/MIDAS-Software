#include"TCAL9539.h"
#include<Wire.h>

#define WIRE Wire


static constexpr uint8_t REG_INPUT0 = 0x0;
static constexpr uint8_t REG_INPUT1 = 0x1;
static constexpr uint8_t REG_OUTPUT0 = 0x2;
static constexpr uint8_t REG_OUTPUT1 = 0x3;
static constexpr uint8_t POLARITY_INVERSION0 = 0x4;
static constexpr uint8_t POLARITY_INVERSION1 = 0x5;
static constexpr uint8_t REG_CONFIG0 = 0x6;
static constexpr uint8_t REG_CONFIG1 = 0x7;

bool TCAL9539Init(){
    uint8_t addrs[] = {GPIO0_ADDRESS, GPIO1_ADDRESS, GPIO2_ADDRESS};

    for(uint8_t addr : addrs){
        Serial.print("Testing ");
        Serial.println(addr);
        WIRE.beginTransmission(addr);
        WIRE.write(REG_OUTPUT0);
        if(!WIRE.endTransmission()){
            Serial.println("Failed at endTransmission");
            return false;
        }
        int ct = WIRE.requestFrom(addr, 1);
        if(ct != 1){
            Serial.println("Failed at requestFrom");
            return false;
        }
        int v = WIRE.read();
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

    uint8_t current_state = pin_state[addr.gpio_id][addr.port_idx];
    if(mode == HIGH){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == LOW){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }

    WIRE.beginTransmission(addr.gpio_address);
    WIRE.write(REG_OUTPUT0 + addr.port_idx);
    WIRE.write(current_state);
    pin_state[addr.gpio_id][addr.port_idx] = current_state;
    if(!WIRE.endTransmission(true)){
        return GpioError::I2CError;
    }

    return GpioError::NoError;
}

GpioReadResult gpioDigitalRead(GpioAddress addr){
    if(!addr.is_valid) {
        return GpioReadResult{.value=LOW,.error=GpioError::InvalidPinError};
    }

    WIRE.beginTransmission(addr.gpio_address);
    WIRE.write(REG_INPUT0 + addr.port_idx);
    if(!WIRE.endTransmission(true)){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }
    int ct = WIRE.requestFrom(addr.gpio_address, 1);
    if(ct != 1){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }
    
    uint8_t val = WIRE.read();

    return GpioReadResult{.value=(val & (1 << addr.pin_offset)) != 0,.error=GpioError::NoError};
}

GpioError gpioPinMode(GpioAddress addr, int mode){
    if(!addr.is_valid){
        return GpioError::NoError;
    }

    uint8_t current_state = pin_config[addr.gpio_id][addr.port_idx];

    if(mode == INPUT){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == OUTPUT){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        return GpioError::InvalidModeError;
    }

    GpioError err = gpioDigitalWrite(addr, LOW); //set pin low as default state
    // if(err != GpioError::NoError){
    //     return err;
    // }

    WIRE.beginTransmission(addr.gpio_address);
    WIRE.write(REG_CONFIG0 + addr.port_idx);
    WIRE.write(current_state);
    pin_config[addr.gpio_id][addr.port_idx] = current_state;

    if(!WIRE.endTransmission(true)){
        return GpioError::I2CError;
    }

    return GpioError::NoError;
}

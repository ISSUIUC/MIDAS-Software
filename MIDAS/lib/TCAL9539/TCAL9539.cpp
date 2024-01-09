#include"TCAL9539.h"
#include<WIRE.h>

#define WIRE Wire
static constexpr uint8_t GPIO0_ADDRESS = 0x74;
static constexpr uint8_t GPIO1_ADDRESS = 0x75;
static constexpr uint8_t GPIO2_ADDRESS = 0x77;

static constexpr uint8_t REG_INPUT0 = 0x0;
static constexpr uint8_t REG_INPUT1 = 0x1;
static constexpr uint8_t REG_OUTPUT0 = 0x2;
static constexpr uint8_t REG_OUTPUT1 = 0x3;
static constexpr uint8_t POLARITY_INVERSION0 = 0x4;
static constexpr uint8_t POLARITY_INVERSION1 = 0x5;
static constexpr uint8_t REG_CONFIG0 = 0x6;
static constexpr uint8_t REG_CONFIG1 = 0x7;

static constexpr int LOW_BIT_COUNT = 8;
static constexpr int HIGH_BIT_COUNT = 16;

bool TCAL9539Init(){
    uint8_t addrs[] = {GPIO0_ADDRESS, GPIO1_ADDRESS, GPIO2_ADDRESS};

    for(uint8_t addr : addrs){
        WIRE.beginTransmission(GPIO0_ADDRESS);
        WIRE.write(REG_OUTPUT0);
        if(!WIRE.endTransmission()){
            return false;
        }
        int ct = WIRE.requestFrom(GPIO0_ADDRESS, 1);
        if(ct != 1){
            return false;
        }
        int v = WIRE.read();
        //REG_OUTPUT0 is set all ones on power up
        if(v != 0xff){
            return false;
        }
    }
    return true;
}

GpioAddress::GpioAddress(int gpio_id, int pin_id){
    this->gpio_id = gpio_id;
    if(gpio_id == 0){
        gpio_address = GPIO0_ADDRESS;
    } else if(gpio_id == 1){
        gpio_address = GPIO1_ADDRESS;
    } else if(gpio_id == 2){
        gpio_address = GPIO2_ADDRESS;
    } else {
        is_valid = false;
        return;
    }
    if(pin_id >= 0 && pin_id < LOW_BIT_COUNT){
        port_idx = 0;
        pin_offset = pin_id;
    } else if(pin_id >= LOW_BIT_COUNT && pin_id < HIGH_BIT_COUNT){
        port_idx = 1;
        pin_offset = pin_id - LOW_BIT_COUNT;
    } else {
        is_valid = false;
        return;
    }
    is_valid = true;
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

    return GpioReadResult{.value=(val & (1 << addr.pin_offset)) != 0,.error=GpioError::I2CError};
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
    if(err != GpioError::NoError){
        return err;
    }

    WIRE.beginTransmission(addr.gpio_address);
    WIRE.write(REG_CONFIG0 + addr.port_idx);
    WIRE.write(current_state);
    pin_config[addr.gpio_id][addr.port_idx] = current_state;

    if(!WIRE.endTransmission(true)){
        return GpioError::I2CError;
    }

    return GpioError::NoError;
}

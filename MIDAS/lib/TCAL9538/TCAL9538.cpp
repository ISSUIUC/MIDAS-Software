#include"TCAL9538.h"
#include<Wire.h>

static constexpr uint8_t REG_INPUT = 0x0;
static constexpr uint8_t REG_OUTPUT = 0x1;
static constexpr uint8_t POLARITY_INVERSION = 0x2;
static constexpr uint8_t REG_CONFIG = 0x3;

// get the correct I2C object if multiple
TwoWire& tcal_get_wire_by_id(int index) {
    if(I2C_BUS_Map[index] == 0) {
        return Wire; 
    } 
    // add cases as needed
    // else {
    //     return Wire1;
    // }
    return Wire;
}

bool TCAL9538Init(int reset_pin){
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
    delay(1);
    digitalWrite(reset_pin, LOW);
    delay(1);
    digitalWrite(reset_pin, HIGH);

    uint8_t addrs[] = {GPIO0_ADDRESS}; // add more addresses as needed
    
    for(int i = 0; i < sizeof(addrs)/sizeof(addrs[0]); i++){
        TwoWire& wire = tcal_get_wire_by_id(i);
        uint8_t addr = addrs[i];

        // remove this when done
        Serial.print("Testing ");
        Serial.println(addr);

        wire.beginTransmission(addr);
        wire.write(REG_OUTPUT);
        if(wire.endTransmission() != 0){
            Serial.print("Failed at endTransmission ");
            Serial.println(addr);
            return false;
        }
        int ct = wire.requestFrom(addr, 1);
        if(ct != 1){
            Serial.print("Failed at requestFrom ");
            Serial.println(addr);
            Serial.println(ct);
            return false;
        }
        int v = wire.read();
        //REG_OUTPUT is set all ones on power up
        if(v != 0xff){
            Serial.println("Failed at REG_OUTPUT");
            return false;
        }
    }
    return true;
}

// adjust index count to reflect number of expanders and add additional 0xff
static uint8_t pin_state[1] = {0xff};
static uint8_t pin_config[1] = {0xff};

GpioError gpioDigitalWrite(GpioAddress addr, int mode){


    if(!addr.is_valid) {
        return GpioError::InvalidPinError;
    }

    TwoWire& wire = tcal_get_wire_by_id(addr.gpio_id);

    uint8_t current_state = pin_state[addr.gpio_id];
    if(mode == HIGH){
        current_state |= (1 << addr.pin_id);
    } else if(mode == LOW){
        current_state &= ~(1 << addr.pin_id);
    } else {
        return GpioError::InvalidModeError;
    }

    wire.beginTransmission(addr.gpio_address);
    wire.write(REG_OUTPUT);
    wire.write(current_state);
    pin_state[addr.gpio_id] = current_state;
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
    wire.write(REG_INPUT);
    if(wire.endTransmission(true) != 0){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }
    int ct = wire.requestFrom(addr.gpio_address, 1);
    if(ct != 1){
        return GpioReadResult{.value=LOW,.error=GpioError::I2CError};
    }

    uint8_t val = wire.read();    
    return GpioReadResult{.value=(val & (1 << addr.pin_id)) != 0,.error=GpioError::NoError};
}

GpioError gpioPinMode(GpioAddress addr, int mode){
    if(!addr.is_valid){
        return GpioError::NoError;
    }

    TwoWire& wire = tcal_get_wire_by_id(addr.gpio_id);

    uint8_t current_state = pin_config[addr.gpio_id];

    if(mode == INPUT){
        current_state |= (1 << addr.pin_id);
    } else if(mode == OUTPUT){
        current_state &= ~(1 << addr.pin_id);
    } else {
        return GpioError::InvalidModeError;
    }
    
    GpioError err = gpioDigitalWrite(addr, LOW);
    if(err != GpioError::NoError){
        return err;
    }

    //wirezero
    wire.beginTransmission(addr.gpio_address);
    wire.write(REG_CONFIG);
    wire.write(current_state);
    pin_config[addr.gpio_id] = current_state;

    if(wire.endTransmission(true) != 0){
        return GpioError::I2CError;
    }
    return GpioError::NoError;
}
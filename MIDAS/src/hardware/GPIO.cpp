#include"GPIO.h"
#include<Wire.h>
#define WIRE Wire
constexpr uint8_t GPIO0_ADDRESS = 0x74;
constexpr uint8_t GPIO1_ADDRESS = 0x75;
constexpr uint8_t GPIO2_ADDRESS = 0x77;

constexpr uint8_t REG_INPUT0 = 0x0;
constexpr uint8_t REG_INPUT1 = 0x1;
constexpr uint8_t REG_OUTPUT0 = 0x2;
constexpr uint8_t REG_OUTPUT1 = 0x3;
constexpr uint8_t POLARITY_INVERSION0 = 0x4;
constexpr uint8_t POLARITY_INVERSION1 = 0x5;
constexpr uint8_t REG_CONFIG0 = 0x6;
constexpr uint8_t REG_CONFIG1 = 0x7;

struct PinAddress {
    uint8_t gpio_addres; //i2c address of expander
    uint8_t port_idx; //0 = bottom 8 bits, 1 = top 8 bits
    uint8_t pin_offset; //[0,7] bit in port, 8 * port_idx + pin_offset = pin idx;
    bool valid; //whether the address is valid
};

static PinAddress decode(GpioAddress pin){
    uint8_t address = 0;
    if(pin.gpio_id == 0){
        address = GPIO0_ADDRESS;
    } else if(pin.gpio_id == 1){
        address = GPIO1_ADDRESS;
    } else if(pin.gpio_id == 2){
        address = GPIO2_ADDRESS;
    } else {
        Serial.println("Err bad gpio index");
        return {.valid = false};
    }
    uint8_t pin_offset;
    uint8_t port_idx;
    if(pin.pin_id >= 0 && pin.pin_id < 8){
        port_idx = 0;
        pin_offset = pin.pin_id;
    } else if(pin.pin_id >= 8 && pin.pin_id < 16){
        port_idx = 1;
        pin_offset = pin.pin_id - 8;
    } else {
        Serial.println("Err bad pin index");
        return {.valid = false};
    }
    return {.gpio_addres = address, .port_idx=port_idx, .pin_offset=pin_offset, .valid=true};
}

static uint8_t pin_state[2][2] = {{0xff,0xff},{0xff,0xff}};
static uint8_t pin_config[2][2] = {{0xff,0xff},{0xff,0xff}};

void digitalWrite(GpioAddress pin, int mode){
    PinAddress addr = decode(pin);
    if(!addr.valid) return;

    uint8_t current_state = pin_state[pin.gpio_id][addr.port_idx];
    if(mode == HIGH){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == LOW){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        Serial.println("Err bad mode");
        return;
    }

    WIRE.beginTransmission(addr.gpio_addres);
    WIRE.write(REG_OUTPUT0 + addr.port_idx);
    WIRE.write(current_state);
    if(!Wire.endTransmission(true)){
        Serial.println("Err i2c transmission failed");
        return;
    }

    pin_state[pin.gpio_id][addr.port_idx] = current_state;
}

int digitalRead(GpioAddress pin){
    PinAddress addr = decode(pin);
    if(!addr.valid) return LOW;

    WIRE.beginTransmission(addr.gpio_addres);
    WIRE.write(REG_INPUT0 + addr.port_idx);
    if(!Wire.endTransmission(true)){
        Serial.println("Err i2c transmission failed");
        return LOW;
    }
    int ct = WIRE.requestFrom(addr.gpio_addres, 1);
    if(ct != 1){
        Serial.println("Err i2c transmission failed");
        return LOW;
    }
    uint8_t val = WIRE.read();

    return val & (1 << addr.pin_offset);
}

void pinMode(GpioAddress pin, int mode){
    PinAddress addr = decode(pin);
    if(!addr.valid) return;

    uint8_t current_state = pin_config[pin.gpio_id][addr.port_idx];

    if(mode == INPUT){
        current_state |= (1 << addr.pin_offset);
    } else if(mode == OUTPUT){
        current_state &= ~(1 << addr.pin_offset);
    } else {
        Serial.println("Err invalid mode");
        return;
    }

    WIRE.beginTransmission(addr.gpio_addres);
    WIRE.write(REG_CONFIG0 + addr.port_idx);
    WIRE.write(current_state);
    if(!WIRE.endTransmission(true)){
        Serial.println("Err i2c transmission failed");
        return;
    }

    pin_config[pin.gpio_id][addr.port_idx] = current_state;
}

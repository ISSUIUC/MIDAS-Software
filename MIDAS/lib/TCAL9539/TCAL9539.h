#pragma once
#include<Arduino.h>

static constexpr uint8_t GPIO0_ADDRESS = 0x74;
static constexpr uint8_t GPIO1_ADDRESS = 0x75;
static constexpr uint8_t GPIO2_ADDRESS = 0x77;

static constexpr int GPIO_LOW_BIT_COUNT = 8;
static constexpr int GPIO_HIGH_BIT_COUNT = 16;

struct GpioAddress {
    GpioAddress(int gpio_id, int pin_id);
    uint8_t gpio_id; //id of the expander
    uint8_t gpio_address; //i2c address of expander
    uint8_t port_idx; //0 = bottom 8 bits, 1 = top 8 bits
    uint8_t pin_offset; //[0,7] bit in port, 8 * port_idx + pin_offset = pin idx;
    bool is_valid; //whether the address is valid
};


inline GpioAddress::GpioAddress(int gpio_id, int pin_id): gpio_id(gpio_id), gpio_address(0), port_idx(0), pin_offset(0), is_valid(false) {
    if(gpio_id == 0){
        gpio_address = GPIO0_ADDRESS;
    } else if(gpio_id == 1){
        gpio_address = GPIO1_ADDRESS;
    } else if(gpio_id == 2){
        gpio_address = GPIO2_ADDRESS;
    } else {
        return;
    }
    if(pin_id >= 0 && pin_id < GPIO_LOW_BIT_COUNT){
        port_idx = 0;
        pin_offset = pin_id;
    } else if(pin_id >= GPIO_LOW_BIT_COUNT && pin_id < GPIO_HIGH_BIT_COUNT){
        port_idx = 1;
        pin_offset = pin_id - GPIO_LOW_BIT_COUNT;
    } else {
        return;
    }
    is_valid = true;
};

enum class GpioError {
    NoError,
    I2CError,
    InvalidPinError,
    InvalidModeError,
};

struct GpioReadResult {
    bool value;
    GpioError error;
};

bool TCAL9539Init();
GpioError gpioPinMode(GpioAddress pin, int mode);
GpioError gpioDigitalWrite(GpioAddress pin, int mode);
GpioReadResult gpioDigitalRead(GpioAddress pin);

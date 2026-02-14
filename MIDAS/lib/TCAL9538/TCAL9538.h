#pragma once
#include<Arduino.h>

// add more addresses as needed
static constexpr uint8_t GPIO0_ADDRESS = 0x70;

// adjust this if the expanders are on different I2C buses
static constexpr int I2C_BUS_Map[] = {0};

static constexpr int GPIO_BIT_COUNT = 8;

struct GpioAddress {
    GpioAddress(int gpio_id, int pin_id);
    uint8_t gpio_id; //id of the expander
    uint8_t gpio_address; //i2c address of expander
    uint8_t pin_id; // pin_id 0-7
    bool is_valid; //whether the address is valid
};


inline GpioAddress::GpioAddress(int gpio_id, int pin_id): gpio_id(gpio_id), gpio_address(0), pin_id(pin_id), is_valid(false) {
    if(gpio_id == 0){
        gpio_address = GPIO0_ADDRESS;
    } 
    // add more cases as needed
    else {
        return;
    }
    if(pin_id < 0 || pin_id >= GPIO_BIT_COUNT){
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

bool TCAL9538Init(int reset_pin);

/**
 * pin: which pin to write or read signal from
 * mode: the mode of the pin
 */
GpioError gpioPinMode(GpioAddress pin, int mode);

GpioError gpioDigitalWrite(GpioAddress pin, int mode);

GpioReadResult gpioDigitalRead(GpioAddress pin);
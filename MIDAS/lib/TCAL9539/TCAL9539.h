#pragma once
#include<Arduino.h>

struct GpioAddress {
    GpioAddress(int gpio_id, int pin_id);
    uint8_t gpio_id;
    uint8_t gpio_address; //i2c address of expander
    uint8_t port_idx; //0 = bottom 8 bits, 1 = top 8 bits
    uint8_t pin_offset; //[0,7] bit in port, 8 * port_idx + pin_offset = pin idx;
    bool is_valid; //whether the address is valid
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

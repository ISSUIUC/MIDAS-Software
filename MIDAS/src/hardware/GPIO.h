#pragma once
#include<Arduino.h>

struct GpioAddress {
    int gpio_id;
    int pin_id;
};

constexpr GpioAddress BNO85_RESET = {.gpio_id = 0, .pin_id = 0};

void pinMode(GpioAddress pin, int mode);
void digitalWrite(GpioAddress pin, int mode);
int digitalRead(GpioAddress pin);


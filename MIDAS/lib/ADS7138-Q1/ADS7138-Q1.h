#pragma once
#include <Arduino.h>

class GPIOADS7138 {
    public:
    bool init();
    void write(unsigned int pin);
    GPIOADS7138();
};
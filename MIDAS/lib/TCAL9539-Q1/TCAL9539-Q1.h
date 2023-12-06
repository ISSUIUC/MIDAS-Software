#pragma once
#include <Arduino.h>

class GPIOTCAL9539 {
    public:
    bool init();
    void write(unsigned int pin, bool state);
    GPIOTCAL9539();
};
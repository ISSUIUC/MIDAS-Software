#pragma once

#include<cstdint>

struct ADCAddress {
    int pin_id;
};

uint16_t analogRead(ADCAddress pin);
bool ADS7138Init();

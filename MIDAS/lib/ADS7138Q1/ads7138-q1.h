#pragma once

#include<cstdint>

struct ADCAddress {
    int pin_id;
};

enum class AdcError {
    NoError,
    I2CError,
    InvalidPinError,
};

struct AdcReadResult {
    uint16_t value;
    AdcError error;
};

AdcReadResult adcAnalogRead(ADCAddress pin);
bool ADS7138Init();

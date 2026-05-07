#pragma once

#include<cstdint>
#include <Wire.h>

#define ADC_PYRO_VOLTAGE_CONVERSION (18.0 / (100.0 + 18.0))
#define ADC_PBATT_VOLTAGE_CONVERSION (100.0 / (100.0 + 560.0))
#define ADC_VBATT_VOLTAGE_CONVERSION (100.0 / (100.0 + 100.0))
#define ADC_NULL_VOLTAGE_CONVERSION 0.0

constexpr float kADCPinConversionFactors[8] = {
    ADC_PYRO_VOLTAGE_CONVERSION,   // Pyro A
    ADC_PYRO_VOLTAGE_CONVERSION,   // Pyro B
    ADC_PBATT_VOLTAGE_CONVERSION,  // Pyro battery
    ADC_NULL_VOLTAGE_CONVERSION,   // Nothing
    ADC_PYRO_VOLTAGE_CONVERSION,   // Pyro C
    ADC_PYRO_VOLTAGE_CONVERSION,   // Pyro D
    ADC_NULL_VOLTAGE_CONVERSION,   // VCAP (unused)
    ADC_VBATT_VOLTAGE_CONVERSION,  // Battery voltage
};

// actual driver for this chip lol
class ADS7138 {
    private:
    uint16_t _ch_readings[8]; // Stored readings for all 8 pins
    uint8_t _i2c_addr;
    TwoWire* _i2c;

    bool i2c_lock();
    void i2c_unlock();
    bool reg_read_single(uint8_t addr, uint8_t& outval);
    bool reg_read_block(uint8_t start_addr, size_t num_reg, uint8_t* out_buf);
    bool reg_write_single(uint8_t addr, uint8_t value);
    bool reg_write_single_bitset(uint8_t addr, uint8_t setmask);

    public:
    bool init(TwoWire* i2c, uint8_t addr);
    bool set_outputs(uint8_t ch_mask);
    void tick(); // Actually does does the channel reading
    float read(uint8_t pin);
};
#include "sensors.h"
#include <MS5611.h>

// #include sensor library

// global static instance of the sensor
MS5611 MS;

ErrorCode BarometerSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Barometer BarometerSensor::read() const {
    MS.read(12);

    float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    float temperature = static_cast<float>(MS.getTemperature() * 0.01);

    Barometer cur_state = {pressure, temperature};

    return cur_state;
}
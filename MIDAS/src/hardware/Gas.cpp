#include "sensors.h"

// #include sensor library
#include <Adafruit_BME680.h>

// global static instance of the sensor
Adafruit_BME680 bme;

/**
 * Initializes the gas sensor, always returns NoError
*/
ErrorCode GasSensor::init() {
    bme.begin();
    return ErrorCode::NoError;
}

/**
 * Checks if the gas sensor is still reading, and returns data
 * @return Gas data packet, all of the data will be -1 if cannot read
*/
Gas GasSensor::read() {
    // read from aforementioned global instance of sensor
    int remaining = bme.remainingReadingMillis();
    float temperature = -1;
    float humidity = -1;
    float pressure = -1;
    float resistance = -1;

    if (remaining == -1) {
        bme.beginReading();
    } else if (remaining == 0) {
        bme.performReading();
        temperature = bme.temperature;
        humidity = bme.humidity;
        pressure = bme.pressure;
        resistance = bme.gas_resistance;
    }

    return Gas{temperature, humidity, pressure, resistance};
}
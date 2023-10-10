#include "sensors.h"

// #include sensor library
#include <Adafruit_BME680.h>
#include <optional>

#define BME688_CS ((int8_t) 0)

// global static instance of the sensor
Adafruit_BME680 bme(BME688_CS);

/**
 * Initializes the gas sensor, always returns NoError
 */
ErrorCode GasSensor::init() {
    if (!bme.begin()) {
        return ErrorCode::GasCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

/**
 * Checks if the gas sensor is still reading, and returns data
 * @return Gas data packet, returns std::nullopt if cannot read
 */
std::optional<Gas> GasSensor::read() {
    int remaining = bme.remainingReadingMillis();

    if (remaining == -1) {
        bme.beginReading();
        return std::nullopt;
    } else if (remaining == 0) {
        bme.performReading();
        bme.beginReading();
        float temperature = bme.temperature;
        float humidity = bme.humidity;
        float pressure = bme.pressure;
        float resistance = bme.gas_resistance;
        return Gas(temperature, humidity, pressure, resistance);
    } else {
        return std::nullopt;
    }
}
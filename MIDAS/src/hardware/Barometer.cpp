#include "sensors.h"
#include <MS5611.h>

/**
 * @brief Singleton instance of the MS5611 barometric pressure sensor.
 *
 * This object provides the interface for initializing the sensor and
 * retrieving pressure, temperature, and altitude measurements.
 */
MS5611 MS(MS5611_CS);       //singleton object for the MS sensor

/**
 * @brief Initializes the MS5611 barometer.
 *
 * Performs the sensor initialization routine required before any
 * measurements can be taken.
 *
 * @return ErrorCode::NoError if initialization completes successfully.
 */
ErrorCode BarometerSensor::init() {
    MS.init();

    return ErrorCode::NoError;
}

/**
 * @brief Reads the latest barometer measurements.
 *
 * Performs a pressure conversion, retrieves the compensated pressure
 * and temperature from the sensor, and computes the estimated altitude
 * using the library's extended atmospheric model.
 *
 * @note Altitude is derived from pressure and is therefore relative to
 *       the atmospheric model and local weather conditions.
 *
 * @return A populated Barometer data packet containing:
 *         - Temperature (°C)
 *         - Pressure (Pa)
 *         - Estimated altitude (m)
 */
Barometer BarometerSensor::read() {
    // Perform a pressure conversion using the highest oversampling ratio.
    MS.read(12);

    /*
     * TODO: Switch to latest version of library (0.3.9) when we get hardware to verify.
     * Equation derived from:
     * https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation
     */

    // Retrieve the compensated atmospheric pressure in Pascals.
    uint32_t pressure = MS.getPressure();

    // Retrieve the compensated sensor temperature in degrees Celsius.
    float temperature = MS.getTemperature();

    // Compute the altitude estimate from the measured pressure.
    float altitude = MS.getAltitudeExtendedModel();

    // Package the measurements into a Barometer data packet.
    return Barometer(temperature, pressure, altitude);
}
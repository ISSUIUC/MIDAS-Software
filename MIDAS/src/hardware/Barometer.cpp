#include "sensors.h"
#include <MS5611.h>

MS5611 MS(MS5611_CS);       //singleton object for the MS sensor

/**
 * @brief Initializes barometer, returns NoError
 * 
 * @return Error code
*/
ErrorCode BarometerSensor::init() {
    MS.init();

    return ErrorCode::NoError;
}

/**
 * @brief Reads the pressure and temperature from the MS5611
 * 
 * @return Barometer data packet
*/
Barometer BarometerSensor::read() {
    MS.read(12);

    /*
     * TODO: Switch to latest version of library (0.3.9) when we get hardware to verify
     * Equation derived from https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation
    */
    uint32_t pressure = MS.getPressure(); // Pascals
    float temperature = MS.getTemperature(); // Celcius
    float altitude = MS.getAltitude();
    
    return Barometer(temperature, pressure, altitude);
}

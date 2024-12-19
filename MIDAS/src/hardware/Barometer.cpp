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
     * TODO: Update the altitude algorithm to ensure that it is reliable
    */
    float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03); // getPressure is in milibars so it's milibars * 0.01?
    float temperature = static_cast<float>(MS.getTemperature() * 0.01); // Celcius
    float altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
    
    return Barometer(temperature, pressure, altitude);
}

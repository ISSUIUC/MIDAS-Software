#include "sensors.h"
#include <MS5611.h>

MS5611 MS(MS5611_CS); 

/**
 * Initializes barometer, returns NoError
*/
ErrorCode BarometerSensor::init() {
    MS.init();

    return ErrorCode::NoError;
}

/**
 * Reads the pressure and temperature from the MS5611
 * @return a barometer data packet for the thread to send to the data logger
*/
Barometer BarometerSensor::read() {
    MS.read(12);

    /*
     * TODO: Switch to latest version of library (0.3.9) when we get hardware to verify
    */
    float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    float temperature = static_cast<float>(MS.getTemperature() * 0.01);
    float altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
    return Barometer(temperature, pressure, altitude);
}

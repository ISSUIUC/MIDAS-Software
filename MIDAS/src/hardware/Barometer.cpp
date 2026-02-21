#include "sensors.h"
#include <MS5611.h>

MS5611_SPI MS(MS5611_CS);       //singleton object for the MS sensor

/**
 * @brief Initializes barometer, returns NoError
 * 
 * @return Error code
*/
ErrorCode BarometerSensor::init() {
    if(!MS.begin()) {
        return ErrorCode::BarometerCoultNotBeInitialized;
    }

    if(!MS.isConnected()) {
        return ErrorCode::BarometerCoultNotBeInitialized;
    }

    MS.setOversampling(OSR_ULTRA_HIGH);
    MS.reset(1); // https://github.com/RobTillaart/MS5611/issues/47

    return ErrorCode::NoError;
}

/**
 * @brief Reads the pressure and temperature from the MS5611
 * 
 * @return Barometer data packet
*/
Barometer BarometerSensor::read() {
    MS.read();

    float pressure = MS.getPressure();
    float temperature = MS.getTemperature();
    float altitude = MS.getAltitude();
    
    return Barometer(temperature, pressure, altitude);
}

#include "sensors.h"
#include <MS5611.h>

// global instance of the barometer sensor, using the same files as TARS
MS5611 MS; 

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

    float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
    float temperature = static_cast<float>(MS.getTemperature() * 0.01);

    return Barometer{pressure, temperature};
}
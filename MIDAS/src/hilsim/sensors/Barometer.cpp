#include "sensors.h"

/**
 * Initializes barometer, returns NoError
*/
ErrorCode BarometerSensor::init() {
    return ErrorCode::NoError;
}

/**
 * Reads the pressure and temperature from the MS5611
 * @return a barometer data packet for the thread to send to the data logger
*/
Barometer BarometerSensor::read() {
    return Barometer{};
}
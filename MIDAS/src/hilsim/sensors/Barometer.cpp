#include "sensors.h"
#include "../global_packet.h"
/**
 * Initializes barometer, returns NoError
*/
ErrorCode BarometerSensor::init() {
    return ErrorCode::NoError;
}

/**
 * Reads the pressure and temperature from the MS5611
 * @return a_m_per_s barometer data packet for the thread to send to the data logger
*/
Barometer BarometerSensor::read() {
    return Barometer{global_packet.barometer_temperature,global_packet.barometer_pressure,global_packet.barometer_altitude};
}
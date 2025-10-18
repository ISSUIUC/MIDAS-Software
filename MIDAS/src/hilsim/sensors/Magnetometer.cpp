#include "../sensors.h"
#include "../kal_rocket.h"

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    return GLOBAL_DATA.magnetometer;
}
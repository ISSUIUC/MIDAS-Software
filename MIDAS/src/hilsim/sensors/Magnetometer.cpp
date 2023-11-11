#include "sensors.h"


ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    return Magnetometer{};
}
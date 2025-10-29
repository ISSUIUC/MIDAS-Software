#include "sensors.h"
#include "../kamaji/kal_rocket.h"

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    return GLOBAL_DATA.magnetometer;
}
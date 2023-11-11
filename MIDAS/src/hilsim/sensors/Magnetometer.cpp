#include "sensors.h"
#include "../packet.h"

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    return Magnetometer{global_packet.mag_x,global_packet.mag_y,global_packet.mag_z};
}
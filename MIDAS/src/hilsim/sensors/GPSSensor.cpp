#include "sensors.h"
#include "../global_packet.h"

ErrorCode GPSSensor::init() {
    return ErrorCode::NoError;
}

GPS GPSSensor::read() {
    return GPS{0.f, 0.f, 0.f, 0.f, 0};
}

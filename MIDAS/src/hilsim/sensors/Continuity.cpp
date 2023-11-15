#include "sensors.h"
#include "../global_packet.h"

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    return Continuity{};
}

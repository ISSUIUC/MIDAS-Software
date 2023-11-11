#include "sensors.h"
#include "../packet.h"

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    return Continuity{};
}

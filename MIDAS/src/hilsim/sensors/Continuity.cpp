#include "sensors.h"

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    return Continuity{};
}

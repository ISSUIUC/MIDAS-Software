#include "sensors.h"
#include "../kamaji/kal_rocket.h"

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    return GLOBAL_DATA.continuity;
}

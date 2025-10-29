#include "sensors.h"
#include "../kamaji/kal_rocket.h"

ErrorCode GPSSensor::init() {
    return ErrorCode::NoError;
}

GPS GPSSensor::read() {
    return GLOBAL_DATA.gps;
}

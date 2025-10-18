#include "../sensors.h"
#include "../kal_rocket.h"

ErrorCode GPSSensor::init() {
    return ErrorCode::NoError;
}

GPS GPSSensor::read() {
    return GLOBAL_DATA.gps;
}

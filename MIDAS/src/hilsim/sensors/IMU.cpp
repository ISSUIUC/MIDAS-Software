#include "sensors.h"
#include "../kamaji/kal_rocket.h"

ErrorCode IMUSensor::init() {
    return ErrorCode::NoError;
}

IMU IMUSensor::read() {
    return GLOBAL_DATA.imu;
}

IMU_SFLP IMUSensor::read_sflp() {
    return GLOBAL_DATA.sflp;
}

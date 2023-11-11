#include "sensors.h"
#include "../packet.h"

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSM LowGLSMSensor::read() {
    return LowGLSM{
        global_packet.imu_low_lsm_ax,global_packet.imu_low_lsm_ay,global_packet.imu_low_lsm_az,
        global_packet.imu_low_lsm_gx,global_packet.imu_low_lsm_gy,global_packet.imu_low_lsm_gz,
        };
}
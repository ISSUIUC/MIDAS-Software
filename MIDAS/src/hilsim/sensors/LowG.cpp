#include "sensors.h"
#include "../packet.h"

ErrorCode LowGSensor::init()
{
    return ErrorCode::NoError;
}

LowGData LowGSensor::read()
{
    return LowGData{global_packet.imu_low_ax,global_packet.imu_low_ay,global_packet.imu_low_az};
}
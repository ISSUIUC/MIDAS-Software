#include "sensors.h"
#include "../global_packet.h"

/**
 * Initializes the high G data sensor, returns ErrorCode::CANNOT_INIT_KX134_CS if cannot initialize
*/
ErrorCode HighGSensor::init() {
    return ErrorCode::NoError;
}

/**
 * Reads and returns the data from the sensor
 * @return a_m_per_s HighGData packet with current acceleration in all three axies
*/
HighGData HighGSensor::read() {
    return HighGData{global_packet.imu_high_ax,global_packet.imu_high_ay,global_packet.imu_high_az};
}
#include "../sensors.h"
#include "../kal_rocket.h"

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSM LowGLSMSensor::read() {
    return GLOBAL_DATA.low_g_lsm;
}
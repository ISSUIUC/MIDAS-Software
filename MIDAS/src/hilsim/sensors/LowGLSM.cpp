#include "sensors.h"

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSM LowGLSMSensor::read() {
    return LowGLSM{};
}
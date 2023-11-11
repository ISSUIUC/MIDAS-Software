#include "sensors.h"

ErrorCode LowGSensor::init()
{
    return ErrorCode::NoError;
}

LowGData LowGSensor::read()
{
    return LowGData{};
}
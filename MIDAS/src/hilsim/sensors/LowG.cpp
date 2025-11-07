#include "sensors.h"
#include "../kamaji/kal_rocket.h"

ErrorCode LowGSensor::init()
{
    return ErrorCode::NoError;
}

LowGData LowGSensor::read()
{
    return GLOBAL_DATA.low_g;
}
#pragma once

/** This header provides the "prototypes" for the sensors, their public interfaces.
 * That way, by switching out what .cpp files we compile in the platformio.ini,
 * we can switch whether we're using hardware sensors or emulated sensors.
 */

#include "sensor_data.h"
#include "errors.h"

struct LowGSensor {
    ErrorCode init();
    LowGData read();
};


struct Sensors {
    LowGSensor low_g;
};

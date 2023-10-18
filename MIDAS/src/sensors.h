#pragma once

/** This header provides the "prototypes" for the sensors, their public interfaces.
 * That way, by switching out what .cpp files we compile in the platformio.ini,
 * we can switch whether we're using hardware sensors or emulated sensors.
 */


#if defined(SILSIM)
#include "silsim/emulated_sensors.h"
#elif defined(HILSIM)
#else
#include "hardware/sensors.h"
#endif


struct Sensors {
    LowGSensor low_g;
    HighGSensor high_g;
    BarometerSensor barometer;
    ContinuitySensor continuity;
    VoltageSensor voltage;
    OrientationSensor orientation;
};

#pragma once

/** This header provides the "prototypes" for the sensors, their public interfaces.
 * That way, by switching out what .cpp files we compile in the platformio.ini,
 * we can switch whether we're using hardware sensors or emulated sensors.
 */

<<<<<<< HEAD
#include "sensor_data.h"
#include "errors.h"
#include "hardware/pins.h"

#include "finite-state-machines/fsm_states.h"
#include "finite-state-machines/fsm.h"

#include<optional>

struct LowGSensor {
    ErrorCode init();
    LowGData read();
    void calibrate();
};

struct Gyroscope {
    ErrorCode init();
    GyroscopeData read();
};

struct HighGSensor {
    ErrorCode init();
    HighGData read();
};

struct PyroThread {
    ErrorCode init();
    Pyro tick_lower(FSMState fsm_state);
    Pyro tick_upper(FSMState fsm_state);
};

struct BarometerSensor {
    ErrorCode init();
    Barometer read();
    
};

struct ContinuitySensor {
    ErrorCode init();
    Continuity read();
};

struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

struct OrientationSensor {
    ErrorCode init();
    Orientation read();
};

struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};
=======
#if defined(SILSIM)
#include "silsim/emulated_sensors.h"
#elif defined(HILSIM)
#else
#include "hardware/sensors.h"
#endif
>>>>>>> mvp-sensor-integration

struct Sensors {
    LowGSensor low_g;
    LowGLSMSensor low_g_lsm;
    HighGSensor high_g;
    BarometerSensor barometer;
    ContinuitySensor continuity;
    VoltageSensor voltage;
    OrientationSensor orientation;
    MagnetometerSensor magnetometer;
<<<<<<< HEAD
    PyroThread pyro;
=======
    GPSSensor gps;
>>>>>>> mvp-sensor-integration
};

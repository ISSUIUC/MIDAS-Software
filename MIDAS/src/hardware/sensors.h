#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"

struct LowGSensor {
    ErrorCode init();
    LowGData read();
};

struct HighGSensor {
    ErrorCode init();
    HighGData read();
};

struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

struct BarometerSensor {
    ErrorCode init();
    Barometer read();
};

struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();
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

struct GPSSensor {
    ErrorCode init();
    GPS read();
};

struct Pyro {
    ErrorCode init();
    PyroState tick_lower(FSMState fsm_state, Orientation orientation);
    PyroState tick_upper(FSMState fsm_state, Orientation orientation);
};
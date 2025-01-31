#pragma once

#include "errors.h"
#include "sensor_data.h"

struct LowGSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    LowGData read() { return lowg; }
    LowGData lowg;
};

struct HighGSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    HighGData read() { return highg; }
    HighGData highg;
};

struct MagnetometerSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    Magnetometer read() { return mag; }
    Magnetometer mag;
};

struct BarometerSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    Barometer read() { return barometer; }
    Barometer barometer;
};

struct LowGLSMSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    LowGLSM read() { return lowglsm; }
    LowGLSM lowglsm;
};

struct ContinuitySensor {
    ErrorCode init() { return ErrorCode::NoError; }
    Continuity read() { return continuity; }
    Continuity continuity;
};

struct VoltageSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    Voltage read() { return voltage; }
    Voltage voltage;
};

struct OrientationSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    Orientation read() { return orient; }
    Orientation orient;
};

struct GPSSensor {
    ErrorCode init() { return ErrorCode::NoError; }
    GPS read() { return gps; }
    GPS gps;
};

struct Pyro {
    ErrorCode init();
    PyroState tick(FSMState fsm_state, Orientation orientation);
    // We will have to set this separately
};

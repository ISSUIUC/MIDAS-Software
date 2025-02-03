#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "pins.h"
struct LowGSensor {
    ErrorCode init();
    LowGData read();
    LowGData lowg;
};

struct HighGSensor {
    ErrorCode init();
    HighGData read();
    HighGData highg;
};

struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
    Magnetometer mag;
};

struct BarometerSensor {
    ErrorCode init();
    Barometer read();
    Barometer barometer;
};

struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();
    LowGLSM lowglsm;
};

struct ContinuitySensor {
    ErrorCode init();
    Continuity read();
    Continuity continuity;
};

struct VoltageSensor {
    ErrorCode init();
    Voltage read() ;
    Voltage voltage;
};

struct OrientationSensor {
    ErrorCode init();
    Orientation read();
    Orientation initial_orientation;
    uint8_t initial_flag;
    Orientation orient;
};

struct GPSSensor {
    ErrorCode init();
    GPS read();
    GPS gps;
};

struct Pyro {
    ErrorCode init();
    PyroState tick(FSMState fsm_state, Orientation orientation);
    // We will have to set this separately
};

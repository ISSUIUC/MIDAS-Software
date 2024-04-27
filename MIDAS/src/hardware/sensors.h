#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"

/**
 * @struct LowG interface
*/
struct LowGSensor {
    ErrorCode init();
    LowGData read();
};

/**
 * @struct HighG interface
*/
struct HighGSensor {
    ErrorCode init();
    HighGData read();
};

/**
 * @struct Magnetometer interface
*/
struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

/**
 * @struct Barometer interface
*/
struct BarometerSensor {
    ErrorCode init();
    Barometer read();
};

/**
 * @struct LowGLSM interface
*/
struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();
};

/**
 * @struct Continuity interface
*/
struct ContinuitySensor {
    ErrorCode init();
    Continuity read();
};

/**
 * @struct Voltage interface
*/
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

/**
 * @struct BNO interface
*/
struct OrientationSensor {
    Orientation initial_orientation;
    uint8_t initial_flag;
    ErrorCode init();
    Orientation read();
};

/**
 * @struct GPS interface
*/
struct GPSSensor {
    ErrorCode init();
    GPS read();
};

/**
 * @struct Pyro interface
*/
struct Pyro {
    ErrorCode init();
    PyroState tick(FSMState fsm_state, Orientation orientation);
};

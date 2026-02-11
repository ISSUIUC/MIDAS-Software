#pragma once

#include "errors.h"
#include "sensor_data.h"

struct LowGSensor {
    ErrorCode init();
    LowGData read();
};


struct ImuSensor {
    ErrorCode init();
    IMU
};


struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

struct BarometerSensor {
    ErrorCode init();
    Barometer read();
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
    PyroState tick(FSMState fsm_state, Orientation orientation);
};

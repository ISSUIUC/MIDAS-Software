#pragma once

#include "errors.h"
#include "sensor_data.h"

#include "silsim/simulation/simulation.h"


struct LowGSensor {
    ErrorCode init();
    LowGData read();

    SimulatedRocket* rocket;
};

struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();

    SimulatedRocket* rocket;
};

struct HighGSensor {
    ErrorCode init();
    HighGData read();

    SimulatedRocket* rocket;
};

struct BarometerSensor {
    ErrorCode init();
    Barometer read();

    SimulatedRocket* rocket;
};

struct ContinuitySensor {
    ErrorCode init();
    Continuity read();

    bool should_be_continous;
};

struct VoltageSensor {
    ErrorCode init();
    Voltage read();

    SimulatedRocket* rocket;
};

struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();

    SimulatedRocket* rocket;
};

struct OrientationSensor {
    ErrorCode init();
    Orientation read();

    SimulatedRocket* rocket;
};

struct GPSSensor {
    ErrorCode init();
    GPS read();

    SimulatedRocket* rocket;
};
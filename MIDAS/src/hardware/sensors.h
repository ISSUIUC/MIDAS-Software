#pragma once

#include "errors.h"
#include "sensor_data.h"

struct LowGSensor {
    ErrorCode init();
    LowGData read();
};

struct HighGSensor {
    ErrorCode init();
    HighGData read();
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
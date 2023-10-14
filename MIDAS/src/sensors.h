#pragma once

/** This header provides the "prototypes" for the sensors, their public interfaces.
 * That way, by switching out what .cpp files we compile in the platformio.ini,
 * we can switch whether we're using hardware sensors or emulated sensors.
 */

#include "sensor_data.h"
#include "errors.h"
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

struct GasSensor {
    ErrorCode init();
    std::optional<Gas> read();
};

struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

struct Sensors {
    LowGSensor low_g;
    Gyroscope gyroscope;
    HighGSensor high_g;
    BarometerSensor barometer;
    ContinuitySensor continuity;
    VoltageSensor voltage;
    OrientationSensor orientation;
    GasSensor gas;
    MagnetometerSensor magnetometer;
};



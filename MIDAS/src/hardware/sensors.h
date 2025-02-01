#pragma once

#include <hardware_interface.h>
#include <finite-state-machines/fsm_states.h>

struct LowGSensor final: ISensor<LowGData> {
    ErrorCode init() override;
    LowGData read() override;
};

struct HighGSensor final: ISensor<HighGData> {
    ErrorCode init() override;
    HighGData read() override;
};

struct MagnetometerSensor final: ISensor<MagnetometerData> {
    ErrorCode init() override;
    MagnetometerData read() override;
};

struct BarometerSensor final: ISensor<BarometerData> {
    ErrorCode init() override;
    BarometerData read() override;
};

struct LowGLSMSensor final: ISensor<LowGLSMData> {
    ErrorCode init() override;
    LowGLSMData read() override;
};

struct ContinuitySensor final: ISensor<ContinuityData> {
    ErrorCode init() override;
    ContinuityData read() override;
};

struct VoltageSensor final: ISensor<VoltageData> {
    ErrorCode init() override;
    VoltageData read() override;
};

struct OrientationSensor final: ISensor<OrientationData> {
    OrientationData initial_orientation;
    uint8_t initial_flag;
    ErrorCode init() override;
    OrientationData read() override;
};

struct GPSSensor final: ISensor<GPSData> {
    ErrorCode init() override;
    GPSData read() override;
    bool is_leap = false;
};

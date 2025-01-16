#pragma once

#include <hardware_interface.h>

#include "errors.h"
#include "sensor_data.h"

#include "silsim/simulation/simulation.h"


struct LowGSensor final: ISensor<LowGData> {
    ErrorCode init() override;
    LowGData read() override;

    SimulatedRocket** rocket;

    explicit LowGSensor(SimulatedRocket** sim);
};

struct LowGLSMSensor final: ISensor<LowGLSMData> {
    ErrorCode init() override;
    LowGLSMData read() override;

    SimulatedRocket** rocket;

    explicit LowGLSMSensor(SimulatedRocket** sim);
};

struct HighGSensor final: ISensor<HighGData> {
    ErrorCode init() override;
    HighGData read() override;

    SimulatedRocket** rocket;

    explicit HighGSensor(SimulatedRocket** sim);
};

struct BarometerSensor final: ISensor<BarometerData> {
    ErrorCode init() override;
    BarometerData read() override;

    SimulatedRocket** rocket;

    explicit BarometerSensor(SimulatedRocket** sim);
};

struct ContinuitySensor final: ISensor<ContinuityData> {
    ErrorCode init() override;
    ContinuityData read() override;

    explicit ContinuitySensor(SimulatedRocket** sim);
};

struct VoltageSensor final: ISensor<VoltageData> {
    ErrorCode init() override;
    VoltageData read() override;

    SimulatedRocket** rocket;

    explicit VoltageSensor(SimulatedRocket** sim);
};

struct MagnetometerSensor final: ISensor<MagnetometerData> {
    ErrorCode init() override;
    MagnetometerData read() override;

    SimulatedRocket** rocket;

    explicit MagnetometerSensor(SimulatedRocket** sim);
};

struct OrientationSensor final: ISensor<OrientationData> {
    ErrorCode init() override;
    OrientationData read() override;

    SimulatedRocket** rocket;

    explicit OrientationSensor(SimulatedRocket** sim);
};

struct GPSSensor final: ISensor<GPSData> {
    ErrorCode init() override;
    GPSData read() override;

    SimulatedRocket** rocket;

    explicit GPSSensor(SimulatedRocket** sim);
};


struct LedBackend final: ILedBackend {
    void set(LED led, bool on) override { }
};

struct BuzzerBackend final: IBuzzerBackend {
    void init() override { }

    void play_tone(uint32_t frequency) override { }
};

struct PyroBackend final: IPyroBackend {
    ErrorCode init() override;

    void arm_all() override;

    void fire_channel(int channel) override;
};

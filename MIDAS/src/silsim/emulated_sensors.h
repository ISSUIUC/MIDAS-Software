#pragma once

#include <hardware_interface.h>

#include "errors.h"
#include "sensor_data.h"

#include "silsim/simulation/simulation.h"


template<class T>
struct EmulatedSensor : ISensor<T> {
protected:
    RocketState& state;

public:
    explicit EmulatedSensor(RocketState& state) : state(state) { };
};


struct LowGSensor final: EmulatedSensor<LowGData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    LowGData read() override;
};

struct LowGLSMSensor final: EmulatedSensor<LowGLSMData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    LowGLSMData read() override;
};

struct HighGSensor final: EmulatedSensor<HighGData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    HighGData read() override;
};

struct BarometerSensor final: EmulatedSensor<BarometerData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    BarometerData read() override;
};

struct ContinuitySensor final: EmulatedSensor<ContinuityData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    ContinuityData read() override;
};

struct VoltageSensor final: EmulatedSensor<VoltageData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    VoltageData read() override;
};

struct MagnetometerSensor final: EmulatedSensor<MagnetometerData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    MagnetometerData read() override;
};

struct OrientationSensor final: EmulatedSensor<OrientationData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    OrientationData read() override;
};

struct GPSSensor final: EmulatedSensor<GPSData> {
    using EmulatedSensor::EmulatedSensor;
    ErrorCode init() override;
    GPSData read() override;
};


struct LedBackend final: ILedBackend {
    void set(LED led, bool on) override {
        (void) led, (void) on;
    }
};

struct BuzzerBackend final: IBuzzerBackend {
    void init() override { }

    void play_tone(uint32_t frequency) override {
        (void) frequency;
    }
};

struct PyroBackend final: IPyroBackend {
    ErrorCode init() override;

    void arm_all() override;
    void fire_channel(int channel) override;
};

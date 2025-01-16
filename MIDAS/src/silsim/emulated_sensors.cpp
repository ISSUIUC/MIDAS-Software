#include "silsim/emulated_sensors.h"
#include <cmath>

BarometerSensor::BarometerSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode BarometerSensor::init() {
    return ErrorCode::NoError;
}

BarometerData BarometerSensor::read() {
    return { 273.15, 0, (float) (*rocket)->height };
}

LowGSensor::LowGSensor(SimulatedRocket** sim) : rocket(sim) { }

LowGLSMSensor::LowGLSMSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSMData LowGLSMSensor::read() {
    return { .gx = 0, .gy = 0, .gz = 0, .ax = 0, .ay = 0, .az =0 };
}

ContinuitySensor::ContinuitySensor(SimulatedRocket** sim) { }

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

ContinuityData ContinuitySensor::read() {
    return { };
}

HighGSensor::HighGSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode HighGSensor::init() {
    return ErrorCode::NoError;
}

HighGData HighGSensor::read() {
    return { 0, 0, (float) (*rocket)->acceleration };
}

ErrorCode LowGSensor::init() {
    return ErrorCode::NoError;
}

LowGData LowGSensor::read() {
    return { 0, 0, (float) (*rocket)->acceleration };
}

OrientationSensor::OrientationSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode OrientationSensor::init() {
    return ErrorCode::NoError;
}

OrientationData OrientationSensor::read() {
    return {
            .has_data = true,
            .yaw = 0,
            .pitch = 0,
            .roll = 0,
            .orientation_velocity = { .vx = 0, .vy = 0, .vz = (float) (*rocket)->velocity },
            .orientation_acceleration = { .ax = 0, .ay = 0, .az = 0 },
            .linear_acceleration = { .ax = 0, .ay = 0, .az = (float) (*rocket)->acceleration },
            .gx = 0,
            .gy = 0,
            .gz = 0,  // todo I don't know what the g's are
            .magnetometer = { .mx = 0, .my = 0, .mz = 0},
            .temperature = 273.15,
    };
}

VoltageSensor::VoltageSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode VoltageSensor::init() {
    return ErrorCode::NoError;
}

VoltageData VoltageSensor::read() {
    return { .voltage = 9 };
}

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

MagnetometerSensor::MagnetometerSensor(SimulatedRocket** sim) : rocket(sim) { }

MagnetometerData MagnetometerSensor::read() {
    return { .mx = 0, .my = 0, .mz = 0 };
}

GPSSensor::GPSSensor(SimulatedRocket** sim) : rocket(sim) { }

ErrorCode GPSSensor::init() {
    return ErrorCode::NoError;
}

GPSData GPSSensor::read() {
    return { };
}

ErrorCode PyroBackend::init() {
    return NoError;
}
void PyroBackend::arm_all() {

}
void PyroBackend::fire_channel(int channel) {

}

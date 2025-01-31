#include "silsim/emulated_sensors.h"
#include <cmath>

ErrorCode BarometerSensor::init() {
    return ErrorCode::NoError;
}

BarometerData BarometerSensor::read() {
    return { 273.15, 0, (float) state.height };
}

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSMData LowGLSMSensor::read() {
    return { .gx = 0, .gy = 0, .gz = 0, .ax = 0, .ay = 0, .az =0 };
}

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

ContinuityData ContinuitySensor::read() {
    return { };
}

ErrorCode HighGSensor::init() {
    return ErrorCode::NoError;
}

HighGData HighGSensor::read() {
    return { 0, 0, (float) state.acceleration };
}

ErrorCode LowGSensor::init() {
    return ErrorCode::NoError;
}

LowGData LowGSensor::read() {
    return { 0, 0, (float) state.acceleration };
}

ErrorCode OrientationSensor::init() {
    return ErrorCode::NoError;
}

OrientationData OrientationSensor::read() {
    return {
        .has_data = true,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .orientation_velocity = { .vx = 0, .vy = 0, .vz = (float) state.velocity },
        .orientation_acceleration = { .ax = 0, .ay = 0, .az = 0 },
        .linear_acceleration = { .ax = 0, .ay = 0, .az = (float) state.acceleration },
        .gx = 0,
        .gy = 0,
        .gz = 0,  // todo I don't know what the g's are
        .magnetometer = { .mx = 0, .my = 0, .mz = 0},
        .temperature = 273.15,
    };
}

ErrorCode VoltageSensor::init() {
    return ErrorCode::NoError;
}

VoltageData VoltageSensor::read() {
    return { .voltage = 9 };
}

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

MagnetometerData MagnetometerSensor::read() {
    return { .mx = 0, .my = 0, .mz = 0 };
}

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
    (void) channel;
}

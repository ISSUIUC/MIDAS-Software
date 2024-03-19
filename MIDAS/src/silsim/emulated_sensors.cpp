#include "silsim/emulated_sensors.h"
#include <cmath>


ErrorCode BarometerSensor::init() {
    return ErrorCode::NoError;
}

Barometer BarometerSensor::read() {
    return { 273.15, 0, (float) rocket->height };
}

ErrorCode LowGLSMSensor::init() {
    return ErrorCode::NoError;
}

LowGLSM LowGLSMSensor::read() {
    return { .gx = 0, .gy = 0, .gz = 0, .ax = 0, .ay = 0, .az =0 };
}

ErrorCode ContinuitySensor::init() {
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    return { };
}

ErrorCode HighGSensor::init() {
    return ErrorCode::NoError;
}

HighGData HighGSensor::read() {
    return { 0, 0, (float) rocket->acceleration };
}

ErrorCode LowGSensor::init() {
    return ErrorCode::NoError;
}

LowGData LowGSensor::read() {
    return { 0, 0, (float) rocket->acceleration };
}

ErrorCode OrientationSensor::init() {
    return ErrorCode::NoError;
}

Orientation OrientationSensor::read() {
    return {
            .has_data = true,
            .yaw = 0,
            .pitch = 0,
            .roll = 0,
            .orientation_velocity = { .vx = 0, .vy = 0, .vz = (float) rocket->velocity },
            .orientation_acceleration = { .ax = 0, .ay = 0, .az = 0 },
            .linear_acceleration = { .ax = 0, .ay = 0, .az = (float) rocket->acceleration },
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

Voltage VoltageSensor::read() {
    return { .voltage = 9 };
}

ErrorCode MagnetometerSensor::init() {
    return ErrorCode::NoError;
}

Magnetometer MagnetometerSensor::read() {
    return { .mx = 0, .my = 0, .mz = 0 };
}

ErrorCode GPSSensor::init() {
    return ErrorCode::NoError;
}

GPS GPSSensor::read() {
    return { };
}

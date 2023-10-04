#include "sensors.h"

ErrorCode BarometerSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Barometer BarometerSensor::read() {
    // read from aforementioned global instance of sensor
    return Barometer();
}

ErrorCode ContinuitySensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Continuity ContinuitySensor::read() {
    // read from aforementioned global instance of sensor
    return Continuity();
}

ErrorCode HighGSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

HighGData HighGSensor::read() {
    // read from aforementioned global instance of sensor
    return HighGData();
}

ErrorCode LowGSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

LowGData LowGSensor::read() {
    // read from aforementioned global instance of sensor
    return LowGData();
}

ErrorCode OrientationSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Orientation OrientationSensor::read() {
    // read from aforementioned global instance of sensor
    return Orientation();
}

ErrorCode VoltageSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Voltage VoltageSensor::read() {
    // read from aforementioned global instance of sensor
    return Voltage();
}
#pragma once

#include <cmath>
#include <cstdint>

/**
 * @brief
 * This header provides all the implementation for the data that comes from all of the sensors/
 * These structs will be individual packets of data passed between the sensor and the 
 * rocket_state struct, and each will be tagged with a timestamp.
*/

/**
 * @brief First 4 structs are base vector, pos, vel, and accel data to be used elsewhere
*/
struct Vec3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Position {
    float px = 0;
    float py = 0;
    float pz = 0;
};

struct Velocity {
    float vx = 0;
    float vy = 0;
    float vz = 0;

    float get_speed() const {
        return sqrtf(vx * vx + vy * vy + vz * vz);
    }
};

struct Acceleration {
    float ax = 0;
    float ay = 0;
    float az = 0;

    // Get G-Force applied on the rocket
    float get_magnitude() const {
        return sqrtf(ax * ax + ay * ay + az * az);
    }
};

/**
 * @struct euler_t
 * 
 * @brief euler representation of rotation
*/
struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

/**
 * @brief Structs starting here represent specific sensors and the respective data
*/

/**
 * @struct LowGData
 * 
 * @brief data from the LowG sensor
*/
struct LowGData {
    float ax = 0;
    float ay = 0;
    float az = 0;

    LowGData() = default;
    LowGData(float x, float y, float z) : ax(x), ay(y), az(z) {}
};

/**
 * @struct HighGData
 * 
 * @brief data from the HighG sensor
*/
struct HighGData {
    float ax = 0;
    float ay = 0;
    float az = 0;

    HighGData() = default;
    HighGData(float x, float y, float z) : ax(x), ay(y), az(z) {}
};

/**
 * @struct LowGLSMData
 * 
 * @brief data from the Low G LSM sensor
*/
struct LowGLSMData {
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
};

/**
 * @struct BarometerData
 * 
 * @brief data from the barometer
*/
struct BarometerData {
    float temperature = 0; // Temperature in Celcius
    float pressure = 0; // Pressure in millibars
    float altitude = 0; // Altitude in meters (above sea level?)

    BarometerData() = default;
    BarometerData(float t, float p, float a) : temperature(t), pressure(p), altitude(a) {}
};

/**
 * @struct ContinuityData
 * 
 * @brief data about pyro continuity
*/
struct ContinuityData {
    float sense_pyro;
    float pins[4];
};

/**
 * @struct VoltageData
 * 
 * @brief data about battery voltage
*/
struct VoltageData {
    float voltage = 0;
};

/**
 * @struct GPSData
 * 
 * @brief data from the GPS
*/
struct GPSData {
    int32_t latitude = 0;
    int32_t longitude = 0;
    float altitude = 0;
    float speed = 0;
    uint16_t satellite_count = 0;
    // Unix timestamp since 1970
    // This isn't included in the telem packet because this is
    // solely for the SD logger. We do not need to know what time it is
    // when we are recieving telem packets.
    uint32_t time = 0;
};

/**
 * @struct MagnetometerData
 * 
 * @brief data from the magnetometer
*/
struct MagnetometerData {
    float mx = 0.0;
    float my = 0.0;
    float mz = 0.0;
};

/**
 * @struct OrientationData
 * 
 * @brief data from the BNO
*/
struct OrientationData {
    bool has_data = false;

    float yaw = 0;
    float pitch = 0;
    float roll = 0;

    Velocity orientation_velocity;
    Acceleration orientation_acceleration;

    Acceleration linear_acceleration;

    float gx = 0, gy = 0, gz = 0;

    MagnetometerData magnetometer;

    float temperature = 0;
    float pressure = 0;

    float tilt = 0;

    //For yessir.cpp
    euler_t getEuler() const {
        return { .yaw = yaw, .pitch = pitch, .roll = roll };
    }
};

/**
 * @struct KalmanData
 * 
 * @brief data from the Kalman thread
*/
struct KalmanData {
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    float altitude = 0.0;
};

/**
 * @struct PyroChannel
 * 
 * @brief data about a specific pyro channel
*/
struct PyroChannel {
    bool is_armed = false;
    bool is_firing = false;
};

/**
 * @struct PyroState
 * 
 * @brief data regarding all pyro channels
*/
struct PyroState {
    bool is_global_armed = false;
    PyroChannel channels[4];
};

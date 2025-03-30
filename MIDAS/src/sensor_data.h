#pragma once

#include <cmath>
#include <cstdint>

#include "finite-state-machines/fsm_states.h"

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
    float x;
    float y;
    float z;
};

struct Position {
    float px;
    float py;
    float pz;
};

struct Velocity {
    float vx;
    float vy;
    float vz;
};

struct Acceleration {
    float ax;
    float ay;
    float az;
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
    float ax;
    float ay;
    float az;
};

/**
 * @struct HighGData
 * 
 * @brief data from the HighG sensor
*/
struct HighGData {
    float ax;
    float ay;
    float az;
};

/**
 * @struct LowGLSM
 * 
 * @brief data from the Low G LSM sensor
*/
struct LowGLSMData {
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
};

/**
 * @struct Barometer
 * 
 * @brief data from the barometer
*/
struct BarometerData {
    float temperature; // Temperature in Celcius
    float pressure; // Pressure in millibars
    float altitude; // Altitude in meters (above sea level?)
};

/**
 * @struct Continuity
 * 
 * @brief data about pyro continuity
*/
struct ContinuityData {
    float pins[4];
};

struct VoltageData {
    float voltage = 0;
    float current = 0;
};

struct GPSData {
    int32_t latitude;
    int32_t longitude;
    float altitude; // Altitude in meters
    float speed; // Speed in meters/second
    uint16_t fix_type;
    // Unix timestamp since 1970
    // This isn't included in the telem packet because this is
    // solely for the SD logger. We do not need to know what time it is
    // when we are recieving telem packets.
    uint32_t time;
};

struct MagnetometerData {
    float mx;
    float my;
    float mz;
};

struct Quaternion {
    float w, x, y, z;

    static float dot(const Quaternion& q1, const Quaternion& q2) {
        return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    }
};

/**
 * @enum Orientation reading type
 */
enum class OrientationReadingType {
    FULL_READING = 0,
    ANGULAR_VELOCITY_UPDATE = 1
};

/**
 * @struct Orientation
 * 
 * @brief data from the BNO
*/
struct OrientationData {
    bool has_data;
    OrientationReadingType reading_type /* = OrientationReadingType::FULL_READING */;

    euler_t euler;

    Velocity orientation_velocity;
    Velocity angular_velocity;

    Acceleration orientation_acceleration;
    Acceleration linear_acceleration;

    float gx, gy, gz;

    MagnetometerData magnetometer;

    float temperature;
    float pressure;

    float tilt;
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

    float altitude;
};

/**
 * @struct PyroState
 * 
 * @brief data regarding all pyro channels
*/
struct PyroState {
    bool is_global_armed;
    bool channel_firing[4];
    /**
     * By convention, the pyro states are as follows:
     * [0] PYRO A / APOGEE
     * [1] PYRO B / MAIN
     * [2] PYRO C / MOTOR
     * [3] PYRO D / AUX
     */
};

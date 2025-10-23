#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

#include "finite-state-machines/fsm_states.h"

//#define CONTINUITY_PIN_COUNT 5

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

    float get_speed() {
        return sqrt(vx * vx + vy * vy + vz * vz);
    }
};

struct Acceleration {
    float ax = 0;
    float ay = 0;
    float az = 0;

    // Get G-Force applied on the rocket
    float get_magnitude() {
        return sqrt(ax * ax + ay * ay + az * az);
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
    LowGData(float x, float y, float z) : ax(x), ay(y), az(z) {};
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
 * @struct LowGLSM
 * 
 * @brief data from the Low G LSM sensor
*/
struct LowGLSM {
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
};

/**
 * @struct Barometer
 * 
 * @brief data from the barometer
*/
struct Barometer {
    float temperature = 0; // Temperature in Celcius
    float pressure = 0; // Pressure in millibars
    float altitude = 0; // Altitude in meters (above sea level?)

    Barometer() = default;
    Barometer(float t, float p, float a) : temperature(t), pressure(p), altitude(a) {}
};

/**
 * @struct Continuity
 * 
 * @brief data about pyro continuity
*/
struct Continuity {
    float pins[4];
};

/**
 * @struct Voltage
 * 
 * @brief data about battery voltage
*/
struct Voltage {
    float voltage = 0;
    float current = 0;
};

/**
 * @struct GPS
 * 
 * @brief data from the GPS
*/
struct GPS {
    int32_t latitude = 0;
    int32_t longitude = 0;
    float altitude = 0; // Altitude in meters
    float speed = 0; // Speed in meters/second
    uint16_t fix_type = 0;
    uint16_t sats_in_view = 0;
    // Unix timestamp since 1970
    // This isn't included in the telem packet because this is
    // solely for the SD logger. We do not need to know what time it is
    // when we are recieving telem packets.
    uint32_t time;
};

/**
 * @struct Magnetometer
 * 
 * @brief data from the magnetometer
*/
struct Magnetometer {
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
struct Orientation {
    bool has_data = false;
    OrientationReadingType reading_type = OrientationReadingType::FULL_READING;

    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    //For yessir.cpp
    euler_t getEuler() const {
        euler_t euler;
        euler.yaw = this->yaw;
        euler.pitch = this->pitch;
        euler.roll = this->roll;
        return euler;
    }


    Velocity orientation_velocity;
    Velocity angular_velocity;

    Velocity getVelocity() const {
        return orientation_velocity;
    }

    Velocity getAngularVelocity() const {
        return angular_velocity;
    }

    Acceleration orientation_acceleration;

    Acceleration linear_acceleration;

    float gx = 0, gy = 0, gz = 0;

    Magnetometer magnetometer;

    float temperature = 0;
    float pressure = 0;

    float tilt = 0;

    Quaternion orientation_quaternion;

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
    bool is_global_armed = false;
    bool channel_firing[4];
    /**
     * By convention, the pyro states are as follows:
     * [0] PYRO A / APOGEE
     * [1] PYRO B / MAIN
     * [2] PYRO C / MOTOR
     * [3] PYRO D / AUX
     */
};



struct CameraData {
    uint8_t camera_state = 255;
    float camera_voltage = 0;
};
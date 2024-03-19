#pragma once

#include <cmath>
#include <cstdint>

#include "finite-state-machines/fsm_states.h"

//#define CONTINUITY_PIN_COUNT 5

/**
 * This header provides all the implementation for the data that comes from all of the sensors/
 * These structs will be individual packets of data passed between the sensor and the 
 * rocket_state struct, and each will be tagged with a timestamp.
*/

/**
 * First 4 structs are base vector, pos, vel, and accel data to be used elsewhere
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

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

/**
 * Structs starting here represent specific sensors and the respective data
*/
struct LowGData {
    float ax = 0;
    float ay = 0;
    float az = 0;

    LowGData() = default;
    LowGData(float x, float y, float z) : ax(x), ay(y), az(z) {};
};

struct HighGData {
    float ax = 0;
    float ay = 0;
    float az = 0;

    HighGData() = default;
    HighGData(float x, float y, float z) : ax(x), ay(y), az(z) {}
};

struct LowGLSM {
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
};

struct Barometer {
    float temperature = 0;
    float pressure = 0;
    float altitude = 0;

    Barometer() = default;
    Barometer(float t, float p, float a) : temperature(t), pressure(p), altitude(a) {}
};

struct Continuity {
    float sense_pyro;
    float pins[4];
};

struct Voltage {
    float voltage = 0;
};

struct GPS {
    float latitude = 0;
    float longitude = 0;
    float altitude = 0;
    float speed = 0;
    uint16_t satellite_count = 0;
};

struct Magnetometer {
    float mx;
    float my;
    float mz;
};

struct Orientation {
    bool has_data = false;

    float yaw = 0;
    float pitch = 0;
    float roll = 0;

    Velocity orientation_velocity;
    Acceleration orientation_acceleration;

    Acceleration linear_acceleration;

    float gx, gy, gz;

    Magnetometer magnetometer;

    float temperature = 0;
    float pressure = 0;

};

struct KalmanData {
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    float altitude;
};


struct PyroChannel {
    bool is_armed = false;
    bool is_firing = false;
};

struct PyroState {
    bool is_global_armed = false;
    PyroChannel channels[4];
};

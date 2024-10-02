#include "sensors.h"
#include "../global_packet.h"
// #include sensor library

// global static instance of the sensor


ErrorCode OrientationSensor::init() {
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    return ErrorCode::NoError;
}

Orientation OrientationSensor::read() {
    // read from aforementioned global instance of sensor
    Velocity ang_vel = Velocity{
        .vx = global_packet.ornt_rollv,
        .vy = global_packet.ornt_pitchv,
        .vz = global_packet.ornt_yawv};
    Acceleration ang_accel = Acceleration{global_packet.ornt_rolla,global_packet.ornt_pitcha,global_packet.ornt_yawa};
    Magnetometer mag = Magnetometer{global_packet.ornt_mx,global_packet.ornt_my,global_packet.ornt_mz};
    Acceleration lin_accel = Acceleration{global_packet.ornt_ax,global_packet.ornt_ay,global_packet.ornt_az};

    return Orientation{
        false, global_packet.ornt_yaw,global_packet.ornt_pitch,global_packet.ornt_rollv,
        ang_vel, ang_accel, lin_accel,
        global_packet.ornt_gx,global_packet.ornt_gy,global_packet.ornt_gz,
        mag,
        global_packet.ornt_temp, global_packet.barometer_pressure
    };
}

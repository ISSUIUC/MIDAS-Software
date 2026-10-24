#include "simulated_sensors.h"

ErrorCode IMUSensor::init() {
    silsim_init("imu");
    silsim_init("sflp");

    return ErrorCode::NoError;
}

IMU IMUSensor::read() {
    std::map<std::string, double> values = silsim_read("imu", 0); // hardcoding 0 just for testing

    IMU reading{};

    reading.highg_acceleration.ax = values.at("highg_acceleration.ax");
    reading.highg_acceleration.ay = values.at("highg_acceleration.ay");
    reading.highg_acceleration.az = values.at("highg_acceleration.az");

    reading.lowg_acceleration.ax = values.at("lowg_acceleration.ax");
    reading.lowg_acceleration.ay = values.at("lowg_acceleration.ay");
    reading.lowg_acceleration.az = values.at("lowg_acceleration.az");

    reading.angular_velocity.vx = values.at("angular_velocity.vx");
    reading.angular_velocity.vy = values.at("angular_velocity.vy");
    reading.angular_velocity.vz = values.at("angular_velocity.vz");

    return reading;
}

IMU_SFLP IMUSensor::read_sflp() {
    std::map<std::string, double> values = silsim_read("sflp", 0); // hardcoding 0 just for tseting
    IMU_SFLP reading{};

    reading.quaternion.w = values.at("quaternion.w");
    reading.quaternion.x = values.at("quaternion.x");
    reading.quaternion.y = values.at("quaternion.y");
    reading.quaternion.z = values.at("quaternion.z");

    reading.gravity.ax = values.at("gravity.ax");
    reading.gravity.ay = values.at("gravity.ay");
    reading.gravity.az = values.at("gravity.az");

    reading.gyro_bias.vx = values.at("gyro_bias.vx");
    reading.gyro_bias.vy = values.at("gyro_bias.vy");
    reading.gyro_bias.vz = values.at("gyro_bias.vz");
    

    return reading;
}

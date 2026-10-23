#pragma once
// A HILSIM version of the RocketData struct without any of the additional wrapping.
#include "flight-systems/sensor_data.h"
#include "flight-systems/rocket_state.h"


struct KRocketData {
public:
    KalmanData kalman;
    AngularKalmanData angular_kalman_data;
    IMU imu;
    IMU_SFLP sflp;
    Barometer barometer;
    PyroState pyro;
    FSMData fsm_state;
    GPS gps;
    Magnetometer magnetometer;
    Voltage voltage;
    CameraData cam_data;
    Latency log_latency;
};

extern KRocketData GLOBAL_DATA;
// A HILSIM version of the RocketData struct without any of the additional wrapping.
#pragma once
#include "sensor_data.h"
#include "rocket_state.h"

struct KRocketData {
public:
    KalmanData kalman;
    LowGData low_g;
    HighGData high_g;
    Barometer barometer;
    LowGLSM low_g_lsm;
    Continuity continuity;
    PyroState pyro;
    FSMState fsm_state;
    GPS gps;
    Magnetometer magnetometer;
    Orientation orientation;
    Voltage voltage;
    
    CommandFlags command_flags;
    uint8_t camera_state = 127;

    Latency log_latency;
};

extern KRocketData GLOBAL_DATA;
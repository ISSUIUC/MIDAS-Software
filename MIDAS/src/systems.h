#pragma once

#include <array>

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"
#include "data_logging.h"
#include "buzzer.h"
#include "led.h"
#include "telemetry.h"
#include "finite-state-machines/fsm.h"
#include "b2b_interface.h"

#if defined(SILSIM)
#include "silsim/emulated_sensors.h"
#elif defined(HILSIM)
#include "TCAL9539.h"
#include "hilsim/sensors/sensors.h"
#else
#include "hardware/sensors.h"
#endif

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    LowGSensor low_g;
    LowGLSMSensor low_g_lsm;
    HighGSensor high_g;
    BarometerSensor barometer;
    ContinuitySensor continuity;
    VoltageSensor voltage;
    OrientationSensor orientation;
    MagnetometerSensor magnetometer;
    Pyro pyro;
    GPSSensor gps;
};

/**
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    LogSink& log_sink;
    BuzzerController buzzer;
    LEDController led;
    Telemetry tlm;
    B2BInterface b2b;
};

[[noreturn]] void begin_systems(RocketSystems* config);

void logger_thread(RocketSystems* arg);
void pyro_thread(RocketSystems* arg);
void kalman_thread(RocketSystems* arg);
void fsm_thread(RocketSystems* arg);
void cam_thread(RocketSystems* arg);
void telemetry_thread(RocketSystems* arg);

void orientation_thread(RocketSystems* arg);
void accelerometers_thread(RocketSystems* arg);
void barometer_thread(RocketSystems* arg);
void gps_thread(RocketSystems* arg);
void voltage_thread(RocketSystems* arg);
void magnetometer_thread(RocketSystems* arg);
void buzzer_thread(RocketSystems* arg);
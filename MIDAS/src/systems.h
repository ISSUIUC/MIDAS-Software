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
#include "hilsim/sensors.h"
#else
#include "hardware/sensors.h"
#endif

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    IMUSensor imu;

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

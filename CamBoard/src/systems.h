#pragma once

#include <array>

#include "sensor_data.h"
#include "hardware/sensors.h"
#include "hal.h"
#include "Buffer.h"
//#include "data_logging.h"
#include "buzzer.h"
#include "led.h"
//#include "telemetry.h"
#include <ACAN2517FD.h>
#include "finite-state-machines/fsm.h"

// #if defined(SILSIM)
// #include "silsim/emulated_sensors.h"
// #elif defined(HILSIM)
// #include "TCAL9539.h"
// #include "hilsim/sensors.h"
// #else
// #include "hardware/sensors.h"
// #endif

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    // LowGSensor low_g;
    // LowGLSMSensor low_g_lsm;
    // HighGSensor high_g;
    // BarometerSensor barometer;
    // ContinuitySensor continuity;
    VoltageSensor voltage;
    CAN can;
    // OrientationSensor orientation;
    // MagnetometerSensor magnetometer;
    // Pyro pyro;
    // GPSSensor gps;
};

/**
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    //LogSink& log_sink;
    BuzzerController buzzer;
    LEDController led;
    //Telemetry tlm;
};

[[noreturn]] void begin_systems(RocketSystems* config);
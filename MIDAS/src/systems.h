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
#include "esp_eeprom.h"
#include "hardware/SDLog.h"
#include "midas_shell.h"

#if defined(SILSIM)
#include "silsim/emulated_sensors.h"
#elif defined(HILSIM)
#include "hilsim/sensors/sensors.h"
#else
#include "hardware/sensors.h"
#endif

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/

//Remove the low_g sensor and high_g sensor, we will be using the imu sensor for the midas mini.

//Barometer (Altitude from Barometer is bugged, need to fix)
struct Sensors {
    IMUSensor imu;

    BarometerSensor barometer;
    VoltageSensor voltage;

    MagnetometerSensor magnetometer;

    Pyro pyro;
    GPSSensor gps;
};

/**
 * @struct RocketSystems
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    LogSink& log_sink;
    MetaLogging meta_logging;
    BuzzerController buzzer;
    LEDController led;
    Telemetry tlm;
    FSM fsm;
    B2BInterface b2b;
    EEPROMController eeprom;
    MShell* shell;
};


[[noreturn]] void begin_systems(RocketSystems* config);

void logger_thread(RocketSystems* arg);
void pyro_thread(RocketSystems* arg);
void kalman_thread(RocketSystems* arg);
void angularkalman_thread(RocketSystems* arg);
void fsm_thread(RocketSystems* arg);
void cam_thread(RocketSystems* arg);
void telemetry_thread(RocketSystems* arg);
void shell_thread(RocketSystems* arg);

void imuthread_thread(RocketSystems* arg);
void barometer_thread(RocketSystems* arg);
void gps_thread(RocketSystems* arg);
void voltage_thread(RocketSystems* arg);
void magnetometer_thread(RocketSystems* arg);
void buzzer_thread(RocketSystems* arg);

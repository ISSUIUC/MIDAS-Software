#pragma once

#include <array>

#include "sensor_data.h"
#include "hardware/sensors.h"
#include "hal.h"
#include "Buffer.h"
#include "buzzer.h"
#include "led.h"
#include "hardware/camera.h"
#include "hardware/CAN.h"
#include "hardware/voltage.cpp"
#include <ACAN2517FD.h>
#include "finite-state-machines/fsm.h"

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    VoltageSensor voltage;
};

/**
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    Sensors sensors;
    RocketData rocket_data;
    BuzzerController buzzer;
    LEDController led;
    Cameras cameras;
    CAN can;
};

[[noreturn]] void begin_systems(RocketSystems* config);
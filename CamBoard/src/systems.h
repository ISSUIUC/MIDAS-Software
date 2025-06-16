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
#include <ACAN2517FD.h>
#include "finite-state-machines/fsm.h"

#define CAMBOARD_I2C_ADDR 0x69

/**
 * @struct Sensors
 * 
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    VoltageSensor voltage;
};

struct cam_state_t {
    volatile bool cam1_on = false;
    volatile bool cam2_on = false;
    volatile bool cam1_rec = false;
    volatile bool cam2_rec = false;
    volatile bool vtx_on = false;
    volatile bool vmux_state = false; // false=CAM1, true=CAM2
    volatile bool cam_ack = false;
};


/**
 * @struct RocketData
 * 
 * @brief holds all information about the rocket, sensors, and controllers
*/
struct RocketSystems {
    //Sensors sensors;        //not used
    RocketData rocket_data;
    BuzzerController buzzer;
    LEDController led;
    Cameras cameras;
    CAN can;
};



[[noreturn]] void begin_systems(RocketSystems* config);
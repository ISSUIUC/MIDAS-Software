#pragma once

#include <cmath>
#include <cstdint>

#include "finite-state-machines/fsm_states.h"
#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>
#include <CANFDMessage.h>

//#define CONTINUITY_PIN_COUNT 5

/**
 * @brief
 * This header provides all the implementation for the data that comes from all of the sensors/
 * These structs will be individual packets of data passed between the sensor and the 
 * rocket_state struct, and each will be tagged with a timestamp.
*/

/**
 * @struct Voltage
 * 
 * @brief data about battery voltage
*/
struct VoltageSense {
    int power = 0;
    int current = 0;
    int temp = 0;
    int voltage = 0;
};


struct MIDASCommands {
    CANFDMessage command;
};
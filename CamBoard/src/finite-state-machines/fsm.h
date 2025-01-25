#pragma once 

#include <numeric>

#include "FreeRTOSConfig.h"
#include "fsm_states.h"
#include "sensor_data.h"
#include "Buffer.h"
#include "rocket_state.h"
#include "systems.h"

/**
 * @class FSM
 * 
 * @brief Contains fsm tick function and timestamps for different events used for future calculations
*/
class FSM {
public:
    FSM() = default;
    FSMState tick_fsm(FSMState &curr_state, RocketSystems* arg);
};
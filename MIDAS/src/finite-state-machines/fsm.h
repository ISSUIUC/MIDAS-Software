#pragma once

//header for fsm governing state transitions and in-flight event control

#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"

struct FSM{
    FSMState tick_fsm(FSMState curr_state);
};
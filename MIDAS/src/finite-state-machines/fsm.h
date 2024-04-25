/**
 * @file fsm.cpp
 * 
 * @brief Header for fsm governing state transitions and in-flight event control
 * 
*/

#pragma once 

#include <numeric>

#include "FreeRTOSConfig.h"
#include "fsm_states.h"
#include "thresholds.h"
#include "sensor_data.h"
#include "Buffer.h"
#include "rocket_state.h"

/**
 * @struct StateEstimate
 * 
 * @brief Holds current alttitude, acceleration, jerk, and speed values based off of averages and derivatives.
*/
struct StateEstimate {
    double altitude;
    double acceleration;
    double jerk;
    double vertical_speed;

    explicit StateEstimate(RocketData& state);
};

/**
 * @class FSM
 * 
 * @brief Contains fsm tick function and timestamps for different events used for future calculations
*/
class FSM {
public:
    FSM() = default;

    FSMState tick_fsm(FSMState& curr_state, StateEstimate state_estimate);

private:
    double launch_time;
    double burnout_time;
    double sustainer_ignition_time;
    double second_boost_time;
    double coast_time;
    double drogue_time;
    double apogee_time;
    double main_time;
    double landed_time;
    double first_separation_time;
};

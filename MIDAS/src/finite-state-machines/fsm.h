#pragma once

// header for fsm governing state transitions and in-flight event control

#include <numeric>

#include "FreeRTOSConfig.h"
#include "fsm_states.h"

#include "thresholds.h"
#include "sensor_data.h"
#include "Buffer.h"
#include "rocket_state.h"


struct StateEstimate {
    double altitude;
    double acceleration;
    double jerk;
    double vertical_speed;

    explicit StateEstimate(RocketState& state);
};


class FSM {
public:
    FSM() = default;

    FSMState tick_fsm_sustainer(FSMState& curr_state, StateEstimate state_estimate);
    FSMState tick_fsm_booster(FSMState& state, StateEstimate state_estimate);

    float getAcceleration(std::array<HighGData, 8>& hg);
    float getAltitude(std::array<Barometer, 8>& bar);
    double getJerk(std::array<HighGData, 8>& hg, double time_delta);
    double getVerticalSpeed(std::array<Barometer, 8>& bar, double time_delta);

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
//    float acceleration;
};

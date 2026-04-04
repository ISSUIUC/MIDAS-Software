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
 * @brief Holds current altitude, acceleration, jerk, and speed values based off of averages and derivatives.
*/
struct StateEstimate {
    double altitude;
    double acceleration;
    double jerk;
    double vertical_speed;

    explicit StateEstimate(RocketData& state);
};

struct FSMPyroAction {
    bool enable;
    FSMState fsm_trigger;
    float delay;
    float max_tilt;
    uint8_t after_motor;
    float launch_t_gt;
    float launch_t_lt;
    float vx_min;
    float vx_max;
};

struct FSMUserThresholds {
    float pyro_fire_t; 
    float main_alt;
    bool cruise_lockout_en;
};

struct FSMConfiguration {
    FSMUserThresholds thresholds;
    FSMPyroAction pyro_actions[4];
    uint8_t version_num;
    uint32_t crc32;
};

/**
 * @class FSM
 * 
 * @brief Contains fsm tick function and timestamps for different events used for future calculations
*/
class FSM {
public:
    FSM() = default;

    FSMState tick_fsm(FSMState& curr_state, StateEstimate state_estimate, CommandFlags& commands);

    const FSMConfiguration& get_cfg() const {return config;};

    bool set_cfg(const FSMConfiguration& new_cfg) {return false;};

private:
    double launch_time;
    double burnout_time;
    double sustainer_ignition_time;
    double second_boost_time;
    double coast_time;
    double drogue_time;
    double apogee_time;
    double main_time;
    double main_deployed_time;
    double landed_time;
    double first_separation_time;
    double pyro_test_entry_time;

    bool landing_lockout_triggered = false;

    FSMConfiguration config;
};

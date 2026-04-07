#pragma once 

#include <numeric>
#include <cstring>

#include "FreeRTOSConfig.h"
#include "fsm_states.h"
#include "thresholds.h"
#include "Buffer.h"
#include "CRC.h"

#ifndef MIDAS_NUM_PYROS
#define MIDAS_NUM_PYROS 4
#endif

struct RocketData;
struct CommandFlags;
struct KalmanData;

#define FSM_CRC_FAIL_STATE ((uint32_t)0)

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

#ifdef FSM_SILSIM_TEST
    StateEstimate(double alt, double accel, double jrk, double vspd)
        : altitude(alt), acceleration(accel), jerk(jrk), vertical_speed(vspd) {}
#endif
};

struct FSMPyroAction {
    bool enable;
    FSMState fsm_trigger;
    float max_tilt;
    uint8_t after_motor;
    float launch_t_gt;
    float launch_t_lt;
    float vx_min;
    float vx_max;

    double delay;

    // Checks that all non-delay conditions are met. Delays are handled in the onboard pyro system.
    bool conditions_met(FSMState fsm_state, float cur_tilt, uint8_t cur_motor, float cur_time_since_launch, float cur_vx) const {
        if (!enable) { return false; }
        if (fsm_state != fsm_trigger) { return false; }
        if(max_tilt != -1 && cur_tilt > max_tilt) { return false; }
        if(after_motor != 0 && cur_motor <= after_motor) { return false; }
        if(launch_t_gt != -1 && cur_time_since_launch < launch_t_gt) { return false; }
        if(launch_t_lt != -1 && cur_time_since_launch > launch_t_lt) { return false; }
        if(vx_min != -1 && cur_vx < vx_min) { return false; }
        if(vx_max != -1 && cur_vx > vx_max) { return false; }

        return true;
    };

    // Checks that conditions that- need to be met the entire "delay" time are met.
    // If these conditions aren't met, the pyro event is reset without consuming it
    bool soft_conditions_met(FSMState fsm_state) const {
        if (!enable) { return false; }
        if (fsm_state != fsm_trigger) { return false; }

        return true;
    }
};

struct FSMUserThresholds {
    float pyro_fire_t; 
    float main_alt;
    bool cruise_lockout_en;
};

struct FSMConfiguration {
    FSMUserThresholds thresholds;
    FSMPyroAction pyro_actions[MIDAS_NUM_PYROS];
    uint8_t version_num;
    uint32_t crc32; // The configuration CRC. If this value is 0, all pyro actions are disabled!

    static uint32_t calculate_crc(const FSMConfiguration& cfg) {
        constexpr size_t FSMConfigurationCRCSize = sizeof(FSMUserThresholds) + (sizeof(FSMPyroAction) * MIDAS_NUM_PYROS) + sizeof(uint8_t);
        uint8_t crc_buf[FSMConfigurationCRCSize];
        uint32_t crc;

        // Copy over data
        size_t ptr = 0;
        memcpy(crc_buf + ptr, &cfg.thresholds, sizeof(FSMUserThresholds)); ptr += sizeof(FSMUserThresholds);
        for(int i = 0; i < MIDAS_NUM_PYROS; i++) {
            memcpy(crc_buf + ptr, &cfg.pyro_actions[i], sizeof(FSMPyroAction)); ptr += sizeof(FSMPyroAction);
        }
        memcpy(crc_buf + ptr, &cfg.version_num, sizeof(uint8_t));

        // Calculate
        return CRC::Calculate(crc_buf, FSMConfigurationCRCSize, CRC::CRC_32());
    }
};

constexpr size_t fsm_config_size = sizeof(FSMConfiguration);
static_assert(fsm_config_size <= 256); 

struct FSMData {
    FSMState state;
    uint8_t current_motor;
};

struct FSMTickData {
    FSMData& cur_state;
    CommandFlags& commands;
    const StateEstimate& state_estimate;
    const KalmanData& kf_data;
    const FSMConfiguration& fsm_config;
    const double cur_time;
};


/**
 * @class FSM
 * 
 * @brief Contains fsm tick function and timestamps for different events used for future calculations
*/
class FSM {
public:
    FSM() = default;

    // FSMState tick_fsm(FSMState& curr_state, StateEstimate state_estimate, CommandFlags& commands);

    FSMData tick_fsm(FSMTickData& fsm_data);

    double get_launch_time() { return launch_time; }

    // Gets the FSM configuration in a non-writeable way.
    const FSMConfiguration& get_cfg() const { return config; }

    // Attempts to set the FSM configuration. Returns TRUE if successful & CRC checks pass.
    bool set_cfg(const FSMConfiguration& new_config);

    // Sets CRC to the fail state, preventing any pyro action.
    void set_crc_fail() { config.crc32 = FSM_CRC_FAIL_STATE; };

private:
    FSMConfiguration config;
    double pyro_test_entry_time;
    double launch_time;
    double apogee_time;
    double time_entered_cur_state;
    double apogee_detect_start = 0;  // (0 = not active)
    double landed_detect_start = 0; // (0 = not active)
    bool cur_state_lockin;
};


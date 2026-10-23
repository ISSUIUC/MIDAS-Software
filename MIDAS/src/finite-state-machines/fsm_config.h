#pragma once

#include "fsm_states.h"

#define MIDAS_NUM_PYROS 4

#define FSM_CRC_FAIL_STATE ((uint32_t)0)

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

    bool conditions_met(FSMState fsm_state, float cur_tilt, uint8_t cur_motor, float cur_time_since_launch, float cur_vx) const;
    bool soft_conditions_met(FSMState fsm_state) const;
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

    static uint32_t calculate_crc(const FSMConfiguration& cfg);
};

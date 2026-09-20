#pragma once

#include <cstdint>

/**
 * @struct CommandFlags
 *
 * @brief Stores the status of commands from telemetry as boolean flags, commands are set whenever the corresponding telemetry command comes in.
 * Works in both directions, say to toggle states based on FSM transitions
 */
struct CommandFlags {
    bool should_reset_kf = false;               // CommandType::RESET_KF
    bool should_transition_safe = false;        // CommandType::SWITCH_TO_SAFE
    bool should_transition_armed = false;       // CommandType::SWITCH_TO_ARMED
    bool should_transition_pyro_test = false;   // CommandType::SWITCH_TO_PYRO_TEST
    bool should_fire_pyro_a = false;            // CommandType::FIRE_PYRO_A
    bool should_fire_pyro_b = false;            // CommandType::FIRE_PYRO_B
    bool should_fire_pyro_c = false;            // CommandType::FIRE_PYRO_C
    bool should_fire_pyro_d = false;            // CommandType::FIRE_PYRO_D
    // FSM Transition commands
    bool FSM_should_set_cam_feed_cam1 = false;  // Triggered at launch (IDLE --> FIRST_BOOST)
    bool FSM_should_power_save = false;         // Triggered after 60 seconds in LANDED state.
    bool FSM_should_swap_camera_feed = false;   // Triggered COAST --> DROGUE
};

struct MErrorFlags {
    uint8_t fsm_crc_err  : 1 = 0;
    uint8_t log_wr_err   : 1 = 0;
    uint8_t log_mr_err   : 1 = 0;
    uint8_t reserved     : 5 = 0;

    uint8_t encode() const {
        uint8_t dat = 0;
        dat |= (fsm_crc_err & 0x01) << 7;
        dat |= (log_wr_err & 0x01) << 6;
        dat |= (log_mr_err & 0x01) << 5;

        return dat;
    }

    void decode(uint8_t dat) {
        fsm_crc_err = (dat >> 7) & 0x01;
        log_wr_err = (dat >> 6) & 0x01;
        log_mr_err = (dat >> 5) & 0x01;
    }
};


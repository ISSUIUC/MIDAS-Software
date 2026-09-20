#pragma once

#include "fsm_states.h"

#define MIDAS_NUM_PYROS 4

// A CRC value of 0 indicates an invalid or uninitialized configuration.
// When detected, all pyro actions are disabled as a safety precaution.
#define FSM_CRC_FAIL_STATE ((uint32_t)0)

/**
 * @struct FSMPyroAction
 *
 * @brief Defines the firing conditions for a single pyro channel.
 *
 * Each pyro output can be independently configured to activate only when
 * all specified flight conditions are satisfied. These conditions include
 * flight state, vehicle orientation, motor count, elapsed launch time,
 * horizontal velocity, and an optional firing delay.
 */
struct FSMPyroAction {
    /// Enables or disables this pyro action.
    bool enable;

    /// FSM state required before this pyro may fire.
    FSMState fsm_trigger;

    /// Maximum allowable tilt angle (degrees) for activation.
    float max_tilt;

    /// Minimum number of completed motor burns required.
    uint8_t after_motor;

    /// Minimum allowable time since launch (seconds).
    float launch_t_gt;

    /// Maximum allowable time since launch (seconds).
    float launch_t_lt;

    /// Minimum allowable horizontal velocity.
    float vx_min;

    /// Maximum allowable horizontal velocity.
    float vx_max;

    /// Delay (seconds) after all conditions become true before firing.
    double delay;

    /**
     * @brief Checks whether all configured firing conditions are satisfied.
     *
     * @param fsm_state Current finite state machine state.
     * @param cur_tilt Current vehicle tilt.
     * @param cur_motor Current motor count or stage.
     * @param cur_time_since_launch Time since launch in seconds.
     * @param cur_vx Current horizontal velocity.
     *
     * @return True if every configured firing condition is met.
     */
    bool conditions_met(FSMState fsm_state, float cur_tilt, uint8_t cur_motor, float cur_time_since_launch, float cur_vx) const;

    /**
     * @brief Checks only the FSM state requirement.
     *
     * Useful for determining whether a pyro is eligible based solely on
     * the current flight state without evaluating other constraints.
     *
     * @param fsm_state Current finite state machine state.
     * @return True if the FSM state requirement is satisfied.
     */
    bool soft_conditions_met(FSMState fsm_state) const;
};

/**
 * @struct FSMUserThresholds
 *
 * @brief User-configurable flight thresholds used by the FSM.
 */
struct FSMUserThresholds {
    /// Time (seconds) that pyro outputs remain active after firing.
    float pyro_fire_t;

    /// Main parachute deployment altitude.
    float main_alt;

    /// Enables cruise-phase deployment lockout.
    bool cruise_lockout_en;
};

/**
 * @struct FSMConfiguration
 *
 * @brief Complete finite state machine configuration.
 *
 * Contains all user-configurable thresholds, pyro action definitions,
 * configuration version information, and an integrity-check CRC.
 */
struct FSMConfiguration {
    /// User-configurable FSM thresholds.
    FSMUserThresholds thresholds;

    /// Configuration for each available pyro channel.
    FSMPyroAction pyro_actions[MIDAS_NUM_PYROS];

    /// Configuration format version.
    uint8_t version_num;

    /**
     * @brief CRC32 checksum of the configuration.
     *
     * If this value equals FSM_CRC_FAIL_STATE (0), the configuration is
     * considered invalid and all pyro actions are disabled.
     */
    uint32_t crc32;

    /**
     * @brief Computes the CRC32 checksum for a configuration.
     *
     * @param cfg Configuration to compute the checksum for.
     * @return CRC32 checksum.
     */
    static uint32_t calculate_crc(const FSMConfiguration& cfg);
};
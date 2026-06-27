#pragma once

#include <numeric>

#include "fsm_config.h"
#include "thresholds.h"
#include "util/Buffer.h"

struct RocketData;
struct CommandFlags;
struct KalmanData;

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

#ifdef FSM_SIMULATOR
    StateEstimate(double alt, double accel, double jrk, double vspd)
        : altitude(alt), acceleration(accel), jerk(jrk), vertical_speed(vspd) {}
#endif
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

    /**
     * @brief Advances the finite state machine by one update cycle.
     *
     * This function evaluates the current flight conditions and command inputs,
     * performs any required state transitions, updates internal timing variables,
     * and returns the resulting FSM state.
     *
     * @param fsm_data Structure containing all data required for FSM evaluation.
     *
     * @return Updated FSM state information.
     */    
    FSMData tick_fsm(FSMTickData& fsm_data);

    /**
     * @brief Returns the recorded launch time.
     *
     * @return Launch time in seconds.
     */
    double get_launch_time() { return launch_time; }

    /**
     * @brief Returns the current FSM configuration.
     *
     * The returned reference is read-only.
     *
     * @return Constant reference to the active configuration.
     */
    const FSMConfiguration& get_cfg() const { return config; }

    /**
    * @brief Loads a new FSM configuration after validating its CRC.
    *
    * The supplied configuration is accepted only if its stored CRC matches
    * the CRC computed from its contents. Invalid configurations are rejected
    * without modifying the currently active configuration.
    *
    * @param new_config Configuration to install.
    *
    * @return True if the configuration was successfully validated and stored.
    */
    bool set_cfg(const FSMConfiguration& new_config);

    /**
     * @brief Forces the configuration into the CRC failure state.
     *
     * Setting the CRC to FSM_CRC_FAIL_STATE disables all configured
     * pyro actions as a safety measure.
     */
    void set_crc_fail() { config.crc32 = FSM_CRC_FAIL_STATE; };

    /**
     * @brief Returns whether the current state has been locked in.
     *
     * @return True if the current state's lock-in period has completed.
     */
    bool get_cur_state_lockin() { return cur_state_lockin; }

private:
    /// Active FSM configuration.
    FSMConfiguration config;

    /// Time at which PYRO_TEST mode was entered.
    double pyro_test_entry_time;

    /// Recorded launch time.
    double launch_time;

    /// Recorded apogee time.
    double apogee_time;

    /// Time the current FSM state was entered.
    double time_entered_cur_state;

    /// Time at which apogee detection began (0 indicates inactive).
    double apogee_detect_start = 0;

    /// Time at which landing detection began (0 indicates inactive).
    double landed_detect_start = 0;

    /// Indicates whether the current state's lock-in period has completed.
    bool cur_state_lockin;
};

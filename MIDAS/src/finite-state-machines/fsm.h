#pragma once

#include <numeric>

#include "fsm_config.h"
#include "thresholds.h"
#include "Buffer.h"

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

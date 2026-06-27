#include <cmath>
#include <cstring>

#include "fsm.h"
#include "thresholds.h"
#include "CRC.h"

bool FSMPyroAction::conditions_met(FSMState fsm_state, float cur_tilt, uint8_t cur_motor, float cur_time_since_launch, float cur_vx) const {
    if (!enable) { return false; }
    if (fsm_state != fsm_trigger) { return false; }
    if (max_tilt != -1 && cur_tilt > max_tilt) { return false; }
    if (after_motor != 0 && cur_motor < after_motor) { return false; }
    if (launch_t_gt != -1 && cur_time_since_launch < launch_t_gt) { return false; }
    if (launch_t_lt != -1 && cur_time_since_launch > launch_t_lt) { return false; }
    if (vx_min != -1 && cur_vx < vx_min) { return false; }
    if (vx_max != -1 && cur_vx > vx_max) { return false; }
    return true;
}

bool FSMPyroAction::soft_conditions_met(FSMState fsm_state) const {
    if (!enable) { return false; }
    if (fsm_state != fsm_trigger) { return false; }
    return true;
}

uint32_t FSMConfiguration::calculate_crc(const FSMConfiguration& cfg) {
    constexpr size_t FSMConfigurationCRCSize = sizeof(FSMUserThresholds) + (sizeof(FSMPyroAction) * MIDAS_NUM_PYROS) + sizeof(uint8_t);
    uint8_t crc_buf[FSMConfigurationCRCSize];

    size_t ptr = 0;
    memcpy(crc_buf + ptr, &cfg.thresholds, sizeof(FSMUserThresholds)); ptr += sizeof(FSMUserThresholds);
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        memcpy(crc_buf + ptr, &cfg.pyro_actions[i], sizeof(FSMPyroAction)); ptr += sizeof(FSMPyroAction);
    }
    memcpy(crc_buf + ptr, &cfg.version_num, sizeof(uint8_t));

    return CRC::Calculate(crc_buf, FSMConfigurationCRCSize, CRC::CRC_32());
}

static_assert(sizeof(FSMConfiguration) <= 256);

#ifdef FSM_SIMULATOR
#include "flight-systems/sensor_data.h"
#include "util/command_flags.h"
#else
#include "flight-systems/rocket_state.h"

/**
 * @brief Helper to calculate the average value of a buffered sensor data
 *
 * @param sensor Buffered sensor struct
 * @param get_item Lambda get function
 *
 * @return Average value
*/
template<typename T, size_t count>
double sensor_average(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    double sum = 0.0;
    for (T& item : arr) {
        sum += get_item(item);
    }
    return sum / count;
}

/**
 * @brief Helper to calculate the derivative over a buffered sensor data
 *
 * @param sensor Buffered sensor struct
 * @param get_item Lambda get function
 *
 * @return Derivative
*/
template<typename T, size_t count>
double sensor_derivative(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    auto times = sensor.template getTimesRecent<count>();
    size_t i = 0;

    double first_average = 0.0;
    double first_average_time = 0.0;
    for (; i < count / 2; i++) {
        first_average += get_item(arr[i]);
        first_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }
    first_average /= (count / 2.0);
    first_average_time /= (count / 2.0);

    double second_average = 0.0;
    double second_average_time = 0.0;
    for (; i < count; i++) {
        second_average += get_item(arr[i]);
        second_average_time += pdTICKS_TO_MS(times[i]) / 1000.0;
    }
    second_average /= (count / 2.0);
    second_average_time /= (count / 2.0);
    return (second_average - first_average) / (second_average_time - first_average_time);
}

/**
 * @brief Populates StateEstimate struct with the correct values for accel, alt, jerk, and speed
*/
StateEstimate::StateEstimate(RocketData& state) {
    acceleration = sensor_average<IMU, 16>(state.imu, [](IMU& data) {
        return (double) data.highg_acceleration.ax; //CHECK WITH DIVIJ/MICHAEL
    });
    altitude = sensor_average<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
    jerk = sensor_derivative<IMU, 16>(state.imu, [](IMU& data) {
        return (double) data.highg_acceleration.ax; //CHECK WITH DIVIJ/MICHAEL
    });
    vertical_speed = sensor_derivative<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
}
#endif

bool FSM::set_cfg(const FSMConfiguration& new_cfg) {
    // Check if the new config has a valid CRC.
    
    uint32_t cfg_crc = new_cfg.crc32;
    uint32_t calculated_crc = FSMConfiguration::calculate_crc(new_cfg);

    if(cfg_crc != calculated_crc) { return false; }
    config = new_cfg;
    return true;
}

/**
 * @brief FSM tick function, which will advance the current state if necessary
 * 
 * @param state current FSM state
 * @param fsm_data Struct containing all data for FSM transitions
 * 
 * @return New FSM State
*/
FSMData FSM::tick_fsm(FSMTickData& fsm_data) {
    double current_time = fsm_data.cur_time;
    FSMData& state_data = fsm_data.cur_state;
    FSMState& state = state_data.state;
    uint8_t& current_motor = state_data.current_motor;
    CommandFlags& commands = fsm_data.commands;
    const StateEstimate& state_estimate = fsm_data.state_estimate;
    const FSMConfiguration& config = fsm_data.fsm_config;
    const KalmanData& kf_data = fsm_data.kf_data;

    switch(state) {
        case FSMState::STATE_SAFE:
            // Deconflict if multiple commands are processed
            if(commands.should_transition_safe) {
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // Only switch to STATE_PYRO_TEST if triggered wirelessly
            if(commands.should_transition_pyro_test) {
                state = FSMState::STATE_PYRO_TEST;
                pyro_test_entry_time = current_time;
                commands.should_transition_pyro_test = false;
            }

            // Only switch to STATE_ARMED if triggered wirelessly.
            if(commands.should_transition_armed) {
                state = FSMState::STATE_ARMED;
                current_motor = 0;
                commands.should_transition_armed = false;
            }

            break;
        case FSMState::STATE_PYRO_TEST:

            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // Switch back to STATE_SAFE after a certain amount of time passes 
            if((current_time - pyro_test_entry_time) > fsms_pt_disarm_t) {
                commands.should_transition_pyro_test = false;
                state = FSMState::STATE_SAFE;
            }

            break;
        case FSMState::STATE_ARMED:

            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_armed = false;
                commands.should_transition_safe = false;
                break;
            }

            // once a significant amount of acceleration is detected change states
            if (state_estimate.acceleration > fsms_boost_xl) {
                launch_time = current_time;                    // Record launch time
                time_entered_cur_state = current_time;         // Record time for false boost detect
                commands.FSM_should_set_cam_feed_cam1 = true;  // Set camera to side cam
                cur_state_lockin = false;                      // Reset "state lock in" flag for boost detect
                state = FSMState::STATE_BOOST;
            }

            break;
        case FSMState::STATE_BOOST:
            // -- BEGIN FALSE MOTOR DETECTION --
            if(!cur_state_lockin && (current_time - time_entered_cur_state) < fsms_boost_lockin_t) {
                // The state has not been locked in yet, so we should check if we still meet the conditions
                if(state_estimate.acceleration < fsms_boost_xl) {
                    state = (current_motor == 0) ? FSMState::STATE_ARMED : FSMState::STATE_COAST;
                    time_entered_cur_state = current_time;
                    break;
                }
            } else {
                if(!cur_state_lockin) {
                    current_motor++; // When "BOOST" gets locked in, we increment the current motor.
                    cur_state_lockin = true;
                }
            }
            // -- END FALSE MOTOR DETECTION --


            // Burnout detection
            if (state_estimate.acceleration < fsms_burnout_xl) {
                time_entered_cur_state = current_time;
                state = FSMState::STATE_COAST;
                cur_state_lockin = false; // Reset "state lock in" flag for false burnout detection
            }
            break;
        case FSMState::STATE_COAST: {
            // if low acceleration is too brief than go on to the previous state

            // -- BEGIN FALSE BURNOUT DETECTION --
            if(!cur_state_lockin && (current_time - time_entered_cur_state) < fsms_burnout_lockin_t) {
                // The state has not been locked in yet, so we should check if we still meet the conditions
                if(state_estimate.acceleration >= fsms_burnout_xl) {
                    state = FSMState::STATE_BOOST;
                    // Note: We do not reset the "time since entering current state" timer, as we're assuming we're still in the previous boost phase.
                    cur_state_lockin = true; // We are going back to a previously "locked-in" state.
                    break;
                }
            } else {
                cur_state_lockin = true;
            }
            // -- END FALSE BURNOUT DETECTION --

            // -- BEGIN NEXT STAGE IGNITION DETECT --
            // After burnout is confirmed, check for another motor ignition (multistage)
            if(cur_state_lockin && state_estimate.acceleration > fsms_boost_xl) {
                time_entered_cur_state = current_time;
                cur_state_lockin = false;
                apogee_detect_start = 0; // Reset apogee detection
                state = FSMState::STATE_BOOST;
                break;
            }
            // -- END NEXT STAGE IGNITION DETECT --

            // -- BEGIN APOGEE DETECT --
            // Condition 1: Vertical speed low
            bool apog_detect_low_speed = (state_estimate.vertical_speed <= fsms_apogee_detect_spd);

            // Condition 2: Cruise lockout
            bool apog_detect_cruise_lockout = (!config.thresholds.cruise_lockout_en || kf_data.velocity.vx <= fsms_cruise_lockout_spd);
            // Note: This evaluates to TRUE (no lockout) if the lockout is disabled, OR if the condition is met

            if (apog_detect_low_speed && apog_detect_cruise_lockout) {
                // Start the consecutive timer if not already running
                if (apogee_detect_start == 0) {
                    apogee_detect_start = current_time;
                }

                // Transition to DROGUE only after conditions are met for fsms_apogee_lockin_t consecutive ms
                if (current_time - apogee_detect_start >= fsms_apogee_lockin_t) {
                    time_entered_cur_state = current_time;
                    apogee_time = current_time;
                    state = FSMState::STATE_DROGUE;
                    apogee_detect_start = 0;

                    commands.FSM_should_swap_camera_feed = true;
                }
            } else {
                // Conditions not met: reset the timer
                apogee_detect_start = 0;
            }
            // -- END APOGEE DETECT --
            break;
        }

        case FSMState::STATE_DROGUE:
            // Simple: If we hit the main threshold, deploy main.
            // Make sure to respect the MAIN_LOCKOUT_T rule.

            if(state_estimate.altitude <= config.thresholds.main_alt && (current_time - time_entered_cur_state) > fsms_main_lockout_t) {
                time_entered_cur_state = current_time;
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN: {
            bool landed_speed = (abs(state_estimate.vertical_speed) <= fsms_landed_detect_spd);
            bool landed_lockout_passed = (current_time - apogee_time) > fsms_landed_t_lockout;

            if (landed_speed && landed_lockout_passed) {
                if (landed_detect_start == 0) {
                    landed_detect_start = current_time;
                }

                if (current_time - landed_detect_start >= fsms_landed_entry_t) {
                    time_entered_cur_state = current_time;
                    cur_state_lockin = false;
                    state = FSMState::STATE_LANDED;
                    landed_detect_start = 0;
                }
            } else {
                landed_detect_start = 0;
            }
            break;
        }

        case FSMState::STATE_LANDED:
            // Landing lock-in
            if((current_time - time_entered_cur_state) > fsms_landed_t) {
                
                if(!cur_state_lockin) {
                    cur_state_lockin = true;
                    commands.FSM_should_power_save = true;
                }

                // Check for any telem transitions
                // Force transtion to safe if requested + clear all transition flags.
                if(commands.should_transition_safe) {
                    state = FSMState::STATE_SAFE;
                    commands.should_transition_pyro_test = false;
                    commands.should_transition_armed = false;
                    commands.should_transition_safe = false;
                }

                break;
            }

            // If we still detect falling movement, go back to MAIN.
            if (abs(state_estimate.vertical_speed) > fsms_landed_detect_spd) {
                state = FSMState::STATE_MAIN;
                time_entered_cur_state = current_time;
            }

            break;
    }

    return state_data;
}
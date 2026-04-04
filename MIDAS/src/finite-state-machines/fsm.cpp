#include <cmath>

#include "fsm.h"
#include "thresholds.h"
#include "rocket_state.h"

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

            // -- BEGIN APOGEE DETECT --
            // Condition 1: Vertical speed low
            bool apog_detect_low_speed = (state_estimate.vertical_speed <= fsms_apogee_detect_spd);

            // Condition 2: Cruise lockout
            bool apog_detect_cruise_lockout = (!config.thresholds.cruise_lockout_en || kf_data.velocity.vx <= fsms_cruise_lockout_spd);
            // Note: This evaluates to TRUE (no lockout) if the lockout is disabled, OR if the condition is met


            if (apog_detect_low_speed && apog_detect_cruise_lockout) {
                // Begin apogee detect
                time_entered_cur_state = current_time;
                // Note: The apogee does not have a "lock in" feature, as it just transitions to DROGUE when the timer expires.
                //       In order to preserve this state's "lock in" in case of erroneous apogee detection, the flag is not reset.
                state = FSMState::STATE_APOGEE;

                // And swap camera feed
                commands.FSM_should_swap_camera_feed = true;
            }
            // -- END APOGEE DETECT --
            break;
        }

        case FSMState::STATE_APOGEE: {
            // We run the apogee detection algorithm above again. Read above for explanation
            bool apog_detect_low_speed = (state_estimate.vertical_speed <= fsms_apogee_detect_spd);
            bool apog_detect_cruise_lockout = (!config.thresholds.cruise_lockout_en || kf_data.velocity.vx <= fsms_cruise_lockout_spd);

            // If either condition is not met, go back to COAST.
            if (!apog_detect_low_speed || !apog_detect_cruise_lockout) {
                state = FSMState::STATE_COAST;
                // Note: We do not reset the lock in flag, as we assume the COAST state is locked in.
                break;
            }

            // If we are still in this state after the apogee lock in timer, go straight to DROGUE, do not pass GO.
            if(current_time - time_entered_cur_state > fsms_apogee_lockin_t) {
                time_entered_cur_state = current_time;
                apogee_time = current_time;
                state = FSMState::STATE_DROGUE;
            }
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

        case FSMState::STATE_MAIN:
            // If we detect very low vertical movement, assume we are LANDED.
            // In case of deployments close to apogee, prevent transitions to LANDED for some time to allow systemt to settle into parachute descent.
            // 200ms debounce timer to prevent ping-ponging between MAIN and LANDED.
            if ((abs(state_estimate.vertical_speed) <= fsms_landed_detect_spd) && (current_time - apogee_time) > fsms_landed_t_lockout && (current_time - time_entered_cur_state) > 200) {
                time_entered_cur_state = current_time;
                cur_state_lockin = false; // Reset flag for landing state lock in logic
                state = FSMState::STATE_LANDED;
            }
            break;

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
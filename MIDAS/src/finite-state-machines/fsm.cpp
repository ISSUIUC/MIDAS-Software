#include <cmath>

#include "fsm.h"
#include "thresholds.h"

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
    acceleration = sensor_average<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    altitude = sensor_average<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
    jerk = sensor_derivative<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    vertical_speed = sensor_derivative<Barometer, 16>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
}


#ifdef IS_SUSTAINER

/**
 * @brief Sustainer FSM tick function, which will advance the current state if necessary
 * 
 * @param state current FSM state
 * @param state_estimate StateEstimate struct for the current estimate for accel, alt, jerk, and speed
 * 
 * @return New FSM State
*/
FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate, CommandFlags& commands) {
    //get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {

        case FSMState::STATE_SAFE:
            // Deconflict if multip commands are processed
            if(commands.should_transition_safe) {
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // Only switch to STATE_PYRO_TEST if triggered wirelessly
            if(commands.should_transition_pyro_test) {
                state = FSMState::STATE_PYRO_TEST;
                pyro_test_entry_time = current_time;
                commands.should_transition_pyro_test = false;
            }

            // Only switch to STATE_IDLE if triggered wirelessly.
            if(commands.should_transition_idle) {
                state = FSMState::STATE_IDLE;
                commands.should_transition_idle = false;
            }

            break;
        case FSMState::STATE_PYRO_TEST:

            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // Switch back to STATE_SAFE after a certain amount of time passes 
            if((current_time - pyro_test_entry_time) > safety_pyro_test_disarm_time) {
                commands.should_transition_pyro_test = false;
                state = FSMState::STATE_SAFE;
            }

            break;

        case FSMState::STATE_IDLE:

            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // once a significant amount of acceleration is detected change states
            if (state_estimate.acceleration > sustainer_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                commands.FSM_should_set_cam_feed_cam1 = true;
                state = FSMState::STATE_FIRST_BOOST;
            }

            break;

        case FSMState::STATE_FIRST_BOOST:
            // if acceleration spike was too brief then go back to idle
            if ((state_estimate.acceleration < sustainer_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time) < sustainer_idle_to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }

            // once acceleartion decreases to a the threshold go on the next state
            if (state_estimate.acceleration < sustainer_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state = FSMState::STATE_BURNOUT;
            }
            break;

        case FSMState::STATE_BURNOUT:
            // if low acceleration is too brief than go on to the previous state
            if ((state_estimate.acceleration >= sustainer_coast_detection_acceleration_threshold) && ((current_time - burnout_time) < sustainer_coast_time)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }

            // if in burnout for long enough then go on to the next state (time transition)
            if ((current_time - burnout_time) > sustainer_coast_time) {
                sustainer_ignition_time = current_time;
                state = FSMState::STATE_SUSTAINER_IGNITION;
            }
            break;

        case FSMState::STATE_SUSTAINER_IGNITION:
            // This state probably does not need a pyro lockout, since we have a back-transition from STATE_SECOND_BOOST
            // another time transition into coast after a certain amount of time
            if ((current_time - sustainer_ignition_time) > sustainer_ignition_to_coast_timer_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
                break;
            }

            // once a high enough acceleration is detected then go to next state
            if (state_estimate.acceleration > sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state = FSMState::STATE_SECOND_BOOST;
            }

            break;

        case FSMState::STATE_SECOND_BOOST:
            // if high accleration is too brief then return to previous state
            if ((state_estimate.acceleration < sustainer_ignition_to_second_boost_acceleration_threshold) && ((current_time - second_boost_time) < sustainer_ignition_to_second_boost_time_threshold)) {
                state = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }

            // if low acceleration detected go to next state
            if (state_estimate.acceleration < sustainer_coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            // if the low acceleration detected was too brief then return to previous state
            if ((state_estimate.acceleration > sustainer_coast_detection_acceleration_threshold) && ((current_time - coast_time) < sustainer_second_boost_to_coast_time_threshold)) {
                state = FSMState::STATE_SECOND_BOOST;
                break;
            }

            // if speed slows down enough then go on to the next stage
            if (state_estimate.vertical_speed <= sustainer_coast_to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state = FSMState::STATE_APOGEE;
                commands.FSM_should_swap_camera_feed = true;
            }
            break;

        case FSMState::STATE_APOGEE:
            // if the slow speed was too brief then return to previous state
            if ((state_estimate.vertical_speed) > sustainer_apogee_backto_coast_vertical_speed_threshold && ((current_time - apogee_time) < sustainer_apogee_check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }

            // transition to next state after a certain amount of time
            if ((current_time - apogee_time) > sustainer_apogee_timer_threshold) {
                drogue_time = current_time;
                state = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            // if detected a sharp change in jerk then go to next state
            // Drogue deploy should ALWAYS stay for at least some time, due to a lack of a back-transition from STATE_DROGUE
            if((current_time - drogue_time) < sustainer_pyro_firing_time_minimum) {
                break;
            }

            if (abs(state_estimate.jerk) > sustainer_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if ((current_time - drogue_time) > sustainer_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            // if altitude low enough then next state
            // Also, wait at least 1 second after drogue deploy to deploy main.
            if (state_estimate.altitude <= sustainer_main_deploy_altitude_threshold && (current_time - drogue_time) > sustainer_main_deploy_delay_after_drogue) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            // Main deploy should ALWAYS stay for at least some time, due to a lack of a back-transition from STATE_DROGUE
            if((current_time - drogue_time) < sustainer_pyro_firing_time_minimum) {
                break;
            }

            // if detected a sharp change in jerk then go to the next state
            if (abs(state_estimate.jerk) > sustainer_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                main_deployed_time = current_time;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if ((current_time - main_time) > sustainer_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            // if slowed down enough then go on to the next state
            if ((abs(state_estimate.vertical_speed) <= sustainer_landed_vertical_speed_threshold) && (current_time - main_deployed_time) > sustainer_main_to_landed_lockout) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:

            if((current_time - landed_time) > sustainer_landed_time_lockout) {
                
                if(!landing_lockout_triggered) {
                    landing_lockout_triggered = true;
                    commands.FSM_should_power_save = true;
                }

                // Check for any telem transitions
                // Force transtion to safe if requested + clear all transition flags.
                if(commands.should_transition_safe) {
                    state = FSMState::STATE_SAFE;
                    commands.should_transition_pyro_test = false;
                    commands.should_transition_idle = false;
                    commands.should_transition_safe = false;
                }

                break;
            }

            // if the slow speed was too brief then return to previous state
            if ((abs(state_estimate.vertical_speed) > sustainer_landed_to_main_vertical_speed_threshold) && ((current_time - landed_time) > sustainer_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;

    }
    return state;
}

#else

/**
 * @brief Booster FSM tick function, which will advance the current state if necessary
 * 
 * This is similar to the previous function but contains less states
 * 
 * @param state current FSM state
 * @param state_estimate StateEstimate struct for the current estimate for accel, alt, jerk, and speed
 * 
 * @return New FSM State
*/
FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate, CommandFlags& commands) {
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
         case FSMState::STATE_SAFE:
            // Deconflict if multip commands are processed
            if(commands.should_transition_safe) {
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // Only switch to STATE_PYRO_TEST if triggered wirelessly
            if(commands.should_transition_pyro_test) {
                state = FSMState::STATE_PYRO_TEST;
                pyro_test_entry_time = current_time;
                commands.should_transition_pyro_test = false;
            }

            // Only switch to STATE_IDLE if triggered wirelessly.
            if(commands.should_transition_idle) {
                state = FSMState::STATE_IDLE;
                commands.should_transition_idle = false;
            }

            break;
        case FSMState::STATE_PYRO_TEST:

            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // Switch back to STATE_SAFE after a certain amount of time passes 
            if((current_time - pyro_test_entry_time) > safety_pyro_test_disarm_time) {
                commands.should_transition_pyro_test = false;
                state = FSMState::STATE_SAFE;
            }

            break;

        case FSMState::STATE_IDLE:
            // Force transtion to safe if requested + clear all transition flags.
            if(commands.should_transition_safe) {
                state = FSMState::STATE_SAFE;
                commands.should_transition_pyro_test = false;
                commands.should_transition_idle = false;
                commands.should_transition_safe = false;
                break;
            }

            // once a significant amount of acceleration is detected change states
            if (state_estimate.acceleration > booster_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                commands.FSM_should_set_cam_feed_cam1 = true;
                state = FSMState::STATE_FIRST_BOOST;
            }

            break;
        case FSMState::STATE_FIRST_BOOST:
            if ((state_estimate.acceleration < booster_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time) < booster_idle_to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }
            if (state_estimate.acceleration < booster_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state = FSMState::STATE_BURNOUT;
            }

            break;

        case FSMState::STATE_BURNOUT:
            if ((state_estimate.acceleration >= booster_coast_detection_acceleration_threshold) && ((current_time - burnout_time) < booster_first_boost_to_burnout_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }

            if ((current_time - burnout_time) > booster_first_boost_to_burnout_time_threshold) {
                first_separation_time = current_time;
                state = FSMState::STATE_FIRST_SEPARATION;
            }
            break;

        case FSMState::STATE_FIRST_SEPARATION:

            // Stage sep should ALWAYS stay for at least some time, due to a lack of a back-transition from STAGE_FIRST_SEPARATION
            if((current_time - first_separation_time) < booster_pyro_firing_time_minimum) {
                break;
            }

            if (abs(state_estimate.jerk) < booster_first_separation_jerk_threshold) {
                state = FSMState::STATE_COAST;
                break;
            }

            if ((current_time - first_separation_time) > booster_first_seperation_time_threshold) {
                state = FSMState::STATE_COAST;
            }

            break;

        case FSMState::STATE_COAST:
            if (state_estimate.vertical_speed <= booster_coast_to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            if (state_estimate.vertical_speed > booster_coast_to_apogee_vertical_speed_threshold && ((current_time - apogee_time) < booster_apogee_check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }

            if ((current_time - apogee_time) > booster_apogee_timer_threshold) {
                drogue_time = current_time;
                state = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:

            // Drogue deploy should ALWAYS stay for at least some time, due to a lack of a back-transition from STATE_DROGUE
            if((current_time - drogue_time) < booster_pyro_firing_time_minimum) {
                break;
            }

            if (abs(state_estimate.jerk) > booster_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }
            if ((current_time - drogue_time) > booster_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            // if altitude low enough then next state
            // Also, wait at least 1 second after drogue deploy to deploy main.
            if (state_estimate.altitude <= booster_main_deploy_altitude_threshold && (current_time - drogue_time) > booster_main_deploy_delay_after_drogue) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:

            // Main deploy should ALWAYS stay for at least some time, due to a lack of a back-transition from STATE_DROGUE
            if((current_time - main_time) < booster_pyro_firing_time_minimum) {
                break;
            }

            if (abs(state_estimate.jerk) > booster_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                main_deployed_time = current_time;
                break;
            }

            if ((current_time - main_time) > booster_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            if (abs(state_estimate.vertical_speed) <= booster_landed_vertical_speed_threshold && (current_time - main_deployed_time) > booster_main_to_landed_lockout) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:

            if((current_time - landed_time) > booster_landed_time_lockout) {

                if(!landing_lockout_triggered) {
                    landing_lockout_triggered = true;
                    commands.FSM_should_power_save = true;
                }

                // Check for any telem transitions
                // Force transtion to safe if requested + clear all transition flags.
                if(commands.should_transition_safe) {
                    state = FSMState::STATE_SAFE;
                    commands.should_transition_pyro_test = false;
                    commands.should_transition_idle = false;
                    commands.should_transition_safe = false;
                }

                break;
            }

            if ((abs(state_estimate.vertical_speed) > booster_landed_to_main_vertical_speed_threshold) && ((current_time - landed_time) > booster_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;

    }
    return state;
}
#endif

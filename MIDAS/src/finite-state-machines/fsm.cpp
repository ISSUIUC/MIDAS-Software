#include <cmath>

#include "fsm.h"
#include "thresholds.h"

// helper functions

template<typename T, size_t count>
double sensor_average(BufferedSensorData<T, count>& sensor, double (* get_item)(T&)) {
    auto arr = sensor.template getBufferRecent<count>();
    double sum = 0.0;
    for (T& item : arr) {
        sum += get_item(item);
    }
    return sum / count;
}

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


StateEstimate::StateEstimate(RocketData& state) {
    acceleration = sensor_average<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    altitude = sensor_average<Barometer, 8>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
    jerk = sensor_derivative<HighGData, 8>(state.high_g, [](HighGData& data) {
        return (double) data.ax;
    });
    vertical_speed = sensor_derivative<Barometer, 8>(state.barometer, [](Barometer& data) {
        return (double) data.altitude;
    });
}


#ifdef IS_SUSTAINER

//added secondStageThresholds arg to pass in struct 
FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate, const secondStageThresholds& thresholds) {
    //get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
        case FSMState::STATE_IDLE:
            // once a significant amount of acceleration is detected change states
            if (state_estimate.acceleration > thresholds.sustainer_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state = FSMState::STATE_FIRST_BOOST;
            }

            break;

        case FSMState::STATE_FIRST_BOOST:
            // if acceleration spike was too brief then go back to idle
            if ((state_estimate.acceleration < thresholds.sustainer_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time) < thresholds.sustainer_idle_to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }

            // once acceleartion decreases to a the threshold go on the next state
            if (state_estimate.acceleration < thresholds.sustainer_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state = FSMState::STATE_BURNOUT;
            }
            break;

        case FSMState::STATE_BURNOUT:
            // if low acceleration is too brief than go on to the previous state
            if ((state_estimate.acceleration >= thresholds.sustainer_coast_detection_acceleration_threshold) && ((current_time - burnout_time) < thresholds.sustainer_first_boost_to_burnout_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }

            // if in burnout for long enough then go on to the next state (time transition)
            if ((current_time - burnout_time) > thresholds.sustainer_first_boost_to_burnout_time_threshold) {
                sustainer_ignition_time = current_time;
                state = FSMState::STATE_SUSTAINER_IGNITION;
            }
            break;

        case FSMState::STATE_SUSTAINER_IGNITION:
            // another time transition into coast after a certain amount of time
            if ((current_time - sustainer_ignition_time) > thresholds.sustainer_ignition_to_coast_timer_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
                break;
            }

            // once a high enough acceleration is detected then go to next state
            if (state_estimate.acceleration > thresholds.sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state = FSMState::STATE_SECOND_BOOST;
            }

            break;

        case FSMState::STATE_SECOND_BOOST:
            // if high accleration is too brief then return to previous state
            if ((state_estimate.acceleration < thresholds.sustainer_ignition_to_second_boost_acceleration_threshold) && ((current_time - second_boost_time) < thresholds.sustainer_ignition_to_second_boost_time_threshold)) {
                state = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }

            // if low acceleration detected go to next state
            if (state_estimate.acceleration < thresholds.sustainer_coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            // if the low acceleration detected was too brief then return to previous state
            if ((state_estimate.acceleration > thresholds.sustainer_coast_detection_acceleration_threshold) && ((current_time - coast_time) < thresholds.sustainer_second_boost_to_coast_time_threshold)) {
                state = FSMState::STATE_SECOND_BOOST;
                break;
            }

            // if speed slows down enough then go on to the next stage
            if (state_estimate.vertical_speed <= thresholds.sustainer_coast_to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            // if the slow speed was too brief then return to previous state
            if ((state_estimate.vertical_speed) > 0 && ((current_time - apogee_time) < thresholds.sustainer_apogee_check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }

            // transition to next state after a certain amount of time
            if ((current_time - apogee_time) > thresholds.sustainer_apogee_timer_threshold) {
                drogue_time = current_time;
                state = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            // if detected a sharp change in jerk then go to next state
            if (abs(state_estimate.jerk) < thresholds.sustainer_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if ((current_time - drogue_time) > thresholds.sustainer_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            // if altitude low enough then next state
            if (state_estimate.altitude <= thresholds.sustainer_main_deploy_altitude_threshold) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            // if detected a sharp change in jerk then go to the next state
            if (abs(state_estimate.jerk) < thresholds.sustainer_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if ((current_time - main_time) > thresholds.sustainer_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            // if slowed down enough then go on to the next state
            if (abs(state_estimate.vertical_speed) <= thresholds.sustainer_landed_vertical_speed_threshold) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            // if the slow speed was too brief then return to previous state
            if ((abs(state_estimate.vertical_speed) > thresholds.sustainer_landed_vertical_speed_threshold) && ((current_time - landed_time) > thresholds.sustainer_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;

    }
    return state;
}

#else

// this is similar to the previous function but contains less states
FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate) {
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
        case FSMState::STATE_IDLE:
            if (state_estimate.acceleration > booster_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
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
            if (abs(state_estimate.jerk) < booster_drogue_jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }
            if ((current_time - drogue_time) > booster_drogue_timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            if (state_estimate.altitude <= booster_main_deploy_altitude_threshold) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            if (abs(state_estimate.jerk) < booster_main_jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }

            if ((current_time - main_time) > booster_main_to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            if (abs(state_estimate.vertical_speed) <= booster_landed_vertical_speed_threshold) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            if ((abs(state_estimate.vertical_speed) > booster_landed_vertical_speed_threshold) && ((current_time - landed_time) > booster_landed_timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;

    }
    return state;
}
#endif

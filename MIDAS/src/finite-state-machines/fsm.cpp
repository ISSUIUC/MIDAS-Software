#include <cmath>

#include "fsm.h"
#include "sg1-4.h"


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

FSMState FSM::tick_fsm(FSMState& state, StateEstimate state_estimate) {
    //get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch (state) {
        case FSMState::STATE_IDLE:
            // once a significant amount of acceleration is detected change states
            if (state_estimate.acceleration > SustainerThresholds::Idle::to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state = FSMState::STATE_FIRST_BOOST;
            }

            break;

        case FSMState::STATE_FIRST_BOOST:
            // if acceleration spike was too brief then go back to idle
            if ((state_estimate.acceleration < SustainerThresholds::Idle::to_first_boost_acceleration_threshold) && ((current_time - launch_time) < SustainerThresholds::Idle::to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }

            // once acceleration decreases to a the threshold go on the next state
            if (state_estimate.acceleration < SustainerThresholds::Coast::detection_acceleration_threshold) {
                burnout_time = current_time;
                state = FSMState::STATE_BURNOUT;
            }
            break;

        case FSMState::STATE_BURNOUT:
            // if low acceleration is too brief than go on to the previous state
            if ((state_estimate.acceleration >= SustainerThresholds::Coast::detection_acceleration_threshold) && ((current_time - burnout_time) < SustainerThresholds::Burnout::to_first_boost_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }

            // if in burnout for long enough then go on to the next state (time transition)
            if ((current_time - burnout_time) > SustainerThresholds::Burnout::to_first_boost_time_threshold) {
                sustainer_ignition_time = current_time;
                state = FSMState::STATE_SUSTAINER_IGNITION;
            }
            break;

        case FSMState::STATE_SUSTAINER_IGNITION:
            // another time transition into coast after a certain amount of time
            if ((current_time - sustainer_ignition_time) > SustainerThresholds::Ignition::to_coast_timer_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
                break;
            }

            // once a high enough acceleration is detected then go to next state
            if (state_estimate.acceleration > SustainerThresholds::Ignition::to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state = FSMState::STATE_SECOND_BOOST;
            }

            break;

        case FSMState::STATE_SECOND_BOOST:
            // if high acceleration is too brief then return to previous state
            if ((state_estimate.acceleration < SustainerThresholds::Ignition::to_second_boost_acceleration_threshold) && ((current_time - second_boost_time) < SustainerThresholds::Ignition::to_second_boost_time_threshold)) {
                state = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }

            // if low acceleration detected go to next state
            if (state_estimate.acceleration < SustainerThresholds::Coast::detection_acceleration_threshold) {
                coast_time = current_time;
                state = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            // if the low acceleration detected was too brief then return to previous state
            if ((state_estimate.acceleration > SustainerThresholds::Coast::detection_acceleration_threshold) && ((current_time - coast_time) < SustainerThresholds::SecondBoost::to_coast_time_threshold)) {
                state = FSMState::STATE_SECOND_BOOST;
                break;
            }

            // if speed slows down enough then go on to the next stage
            if (state_estimate.vertical_speed <= SustainerThresholds::Coast::to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            // if the slow speed was too brief then return to previous state
            if ((state_estimate.vertical_speed) > SustainerThresholds::Apogee::backto_coast_vertical_speed_threshold && ((current_time - apogee_time) < SustainerThresholds::Apogee::check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }

            // transition to next state after a certain amount of time
            if ((current_time - apogee_time) > SustainerThresholds::Apogee::timer_threshold) {
                drogue_time = current_time;
                state = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            // if detected a sharp change in jerk then go to next state
            if (abs(state_estimate.jerk) < SustainerThresholds::Drogue::jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }

            // if no transition after a certain amount of time then just move on to next state
            if ((current_time - drogue_time) > SustainerThresholds::Drogue::timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            // if altitude low enough then next state
            if (state_estimate.altitude <= SustainerThresholds::Main::deploy_altitude_threshold) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            // if detected a sharp change in jerk then go to the next state
            if (abs(state_estimate.jerk) < SustainerThresholds::Main::jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }

            // if no transition after a certain amount of time then just move on to next state
            if ((current_time - main_time) > SustainerThresholds::Main::to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            // if slowed down enough then go on to the next state
            if (abs(state_estimate.vertical_speed) <= SustainerThresholds::Landed::vertical_speed_threshold) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            // if the slow speed was too brief then return to previous state
            if ((abs(state_estimate.vertical_speed) > SustainerThresholds::Landed::vertical_speed_threshold) && ((current_time - landed_time) > SustainerThresholds::Landed::timer_threshold)) {
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
            if (state_estimate.acceleration > BoosterThresholds::Idle::to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state = FSMState::STATE_FIRST_BOOST;
            }

            break;

        case FSMState::STATE_FIRST_BOOST:
            if ((state_estimate.acceleration < BoosterThresholds::Idle::to_first_boost_acceleration_threshold) && ((current_time - launch_time) < BoosterThresholds::Idle::to_first_boost_time_threshold)) {
                state = FSMState::STATE_IDLE;
                break;
            }
            if (state_estimate.acceleration < BoosterThresholds::Coast::detection_acceleration_threshold) {
                burnout_time = current_time;
                state = FSMState::STATE_BURNOUT;
            }

            break;

        case FSMState::STATE_BURNOUT:
            if ((state_estimate.acceleration >= BoosterThresholds::Coast::detection_acceleration_threshold) && ((current_time - burnout_time) < BoosterThresholds::Burnout::to_first_boost_time_threshold)) {
                state = FSMState::STATE_FIRST_BOOST;
                break;
            }

            if ((current_time - burnout_time) > BoosterThresholds::Burnout::to_first_boost_time_threshold) {
                first_separation_time = current_time;
                state = FSMState::STATE_FIRST_SEPARATION;
            }
            break;

        case FSMState::STATE_FIRST_SEPARATION:
            if (abs(state_estimate.jerk) < BoosterThresholds::FirstSeparation::jerk_threshold) {
                state = FSMState::STATE_COAST;
                break;
            }

            if ((current_time - first_separation_time) > BoosterThresholds::FirstSeparation::time_threshold) {
                state = FSMState::STATE_COAST;
            }

            break;

        case FSMState::STATE_COAST:
            if (state_estimate.vertical_speed <= BoosterThresholds::Coast::to_apogee_vertical_speed_threshold) {
                apogee_time = current_time;
                state = FSMState::STATE_APOGEE;
            }
            break;

        case FSMState::STATE_APOGEE:
            if (state_estimate.vertical_speed > BoosterThresholds::Coast::to_apogee_vertical_speed_threshold && ((current_time - apogee_time) < BoosterThresholds::Apogee::check_threshold)) {
                state = FSMState::STATE_COAST;
                break;
            }

            if ((current_time - apogee_time) > BoosterThresholds::Apogee::timer_threshold) {
                drogue_time = current_time;
                state = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;

        case FSMState::STATE_DROGUE_DEPLOY:
            if (abs(state_estimate.jerk) < BoosterThresholds::Drogue::jerk_threshold) {
                state = FSMState::STATE_DROGUE;
                break;
            }
            if ((current_time - drogue_time) > BoosterThresholds::Drogue::timer_threshold) {
                state = FSMState::STATE_DROGUE;
            }

            break;

        case FSMState::STATE_DROGUE:
            if (state_estimate.altitude <= BoosterThresholds::Main::deploy_altitude_threshold) {
                state = FSMState::STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case FSMState::STATE_MAIN_DEPLOY:
            if (abs(state_estimate.jerk) < BoosterThresholds::Main::jerk_threshold) {
                state = FSMState::STATE_MAIN;
                break;
            }

            if ((current_time - main_time) > BoosterThresholds::Main::to_main_deploy_timer_threshold) {
                state = FSMState::STATE_MAIN;
            }
            break;

        case FSMState::STATE_MAIN:
            if (abs(state_estimate.vertical_speed) <= BoosterThresholds::Landed::vertical_speed_threshold) {
                landed_time = current_time;
                state = FSMState::STATE_LANDED;
            }
            break;

        case FSMState::STATE_LANDED:
            if ((abs(state_estimate.vertical_speed) > BoosterThresholds::Landed::vertical_speed_threshold) && ((current_time - landed_time) > BoosterThresholds::Landed::timer_threshold)) {
                state = FSMState::STATE_MAIN;
            }
            break;

    }
    return state;
}
#endif

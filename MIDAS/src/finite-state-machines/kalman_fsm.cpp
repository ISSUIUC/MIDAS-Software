#include <cmath>

#include "kalman_fsm.h"
#include "thresholds.h"
#include "../gnc/ekf.h" // can someone fix import paths?
#include "../gnc/yessir.h" // this too
#include "../sensor_data.h" // this too

void KalmanFSM::tickFSM() {
    KalmanData current_state = ekf.getState();

    //Get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    switch(rocket_state_) {
        case FSMState::STATE_IDLE:
            if (current_state.acceleration > launch_acceleration_threshold) {
                launch_time_ = current_time;
                rocket_state_ = FSMState::FIRST_BOOST;
            }
            break;
        case FSMState::STATE_FIRST_BOOST:
            if ((current_state.acceleration < launch_acceleration_threshold) && ((current_time - launch_time_) < idle_to_first_boost_time_threshold)) {
                rocket_state_ = FSMState::STATE_IDLE;
                break;
            }
            if (current_state.acceleration < coast_detection_threshold) {
                burnout_time = current_time;
                rocket_state_ = FSMState::STATE_BURNOUT;
            }
            break;
        case FSMState::STATE_BURNOUT:
            if ((current_state.acceleration >= coast_detection_threshold) && ((current_time - burnout_time_) < burnout_time_threshold)) {
                rocket_state_ = FSMState::STATE_FIRST_BOOST;
                break;
            }

            if ((current_time - burnout_time_) > coast_time) {
                sustainer_ignition_time_ = current_time;
                rocket_state_ = FSMState::STATE_SUSTAINER_IGNITION;
            }
            break;

        case FSMState::STATE_SUSTAINER_IGNITION:
            if ((current_time - sustainer_ignition_time_) > sustainer_ignition_timeout) {
                coast_time_ = current_time;
                rocket_state_ = FSMState::STATE_COAST;
                break;
            }
            if (current_state.acceleration > sustainer_boost_threshold) {
                second_boost_time = current_time;
                rocket_state_ = FSMState::STATE_SECOND_BOOST;
            }
            break;

        case FSMState::STATE_SECOND_BOOST:
            if ((current_state.acceleration < sustainer_boost_threshold) && ((current_time - second_boost_time_) < second_boost_time_threshold)) {
                rocket_state_ = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }
            if (current_state.acceleration < coast_detection_threshold) {
                coast_time_ = current_time;
                rocket_state_ = FSMState::STATE_COAST;
            }
            break;

        case FSMState::STATE_COAST:
            if (current_state.velocity.vx <= apogee_velocity_threshold) {
                apogee_time_ = current_time;
                rocket_state_ = FSMState::STATE_APOGEE;
            }
            break;
        case FSMState::STATE_APOGEE:
            if ((current_state.velocity.vx > apogee_velocity_threshold) && ((current_time - apogee_time_) < apogee_check_threshold)) {
                rocket_state_ = FSMState::STATE_COAST;
                break;
            }
            if ((current_time - apogee_time_) > apogee_timer_threshold) {
                drogue_time_ = current_time;
                rocket_state_ = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            if ((current_time - drogue_time_) < pyro_firing_time_minimum) {
                break;
            }
            if ((current_time - drogue_time_) > drogue_deploy_timeout) {
                rocket_state_ = FSMState::STATE_DROGUE;
            }
            break;
        case FSMState::STATE_DROGUE:
            if ((current_state.position.x <= main_deploy_altitude) && ((current_time - drogue_time_) > main_deploy_delay)) {
                main_time_ = current_time;
                rocket_state_ = FSMState::STATE_MAIN_DEPLOY;
            }
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            if ((current_time - main_time_) < pyro_firing_time_minimum) {
                break;
            }
            if ((current_time - main_time_) > main_deploy_timeout) {
                main_deployed_time_ = current_time;
                rocket_state_ = FSMState::STATE_MAIN;
            }
            break;
        case FSMState::STATE_MAIN:
            if ((std::abs(current_state.velocity.vx) <= landing_velocity_threshold) && ((current_time - main_deployed_time_) > main_to_landed_lockout)) {
                landed_time_ = current_time;
                rocket_state_ = FSMState::STATE_LANDED;
            } 
            break;
        case FSMState::STATE_LANDED:
            if ((current_time - landed_time_) > landed_time_lockout) {
                if (commands_.should_transition_safe) {
                    rocket_state_ = FSMState::STATE_SAFE;
                }
                break;
            }
            if ((std::abs(current_state.velocity.vx) > landing_velocity_threshold) && 
                ((current_time - landed_time_) > landed_check_threshold)) {
                rocket_state_ = FSMState::STATE_MAIN;
            }
            break;
    }
}
#include <cmath>
#include "kalman_fsm.h"
#include "thresholds.h"
#include "../gnc/ekf.h" // can someone fix import paths?
#include "../gnc/yessir.h" // this too
#include "../sensor_data.h" // this too
double KalmanFSM::getAltitudeAverage(size_t start, size_t len) {
}
double KalmanFSM::getSecondDerivativeAltitudeAverage(size_t start, size_t len) {
}
double KalmanFSM::getAccelerationAverage(size_t start, size_t len) {
}
void KalmanFSM::tick_fsm() {
    KalmanData current_state = ekf.getState();
    //Get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());
    switch(rocket_state_) {
        case FSMState::STATE_IDLE:
            if (current_state.acceleration.ax > sustainer_idle_to_first_boost_acceleration_threshold) {
                launch_time_ = current_time;
                rocket_state_ = FSMState::STATE_FIRST_BOOST;
            }
            break;
        case FSMState::STATE_FIRST_BOOST:
            if ((current_state.acceleration.ax < sustainer_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time_) < booster_idle_to_first_boost_time_threshold)) {
                rocket_state_ = FSMState::STATE_IDLE;
                break;
            }
            if (current_state.acceleration.ax <  booster_coast_detection_acceleration_threshold) {
                burnout_time_ = current_time;
                rocket_state_ = FSMState::STATE_BURNOUT;
            }
            break;
        case FSMState::STATE_BURNOUT:
            if ((current_state.acceleration.ax >= booster_coast_detection_acceleration_threshold) && ((current_time - burnout_time_) < booster_first_seperation_time_threshold)) {
                rocket_state_ = FSMState::STATE_FIRST_BOOST;
                break;
            }
            if ((current_time - burnout_time_) > sustainer_coast_time) {
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
            if (current_state.acceleration.ax > sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time_ = current_time;
                rocket_state_ = FSMState::STATE_SECOND_BOOST;
            }
            break;
        case FSMState::STATE_SECOND_BOOST:
            if ((current_state.acceleration < sustainer_ignition_to_second_boost_acceleration_threshold) && ((current_time - second_boost_time_) < sustainer_second_boost_to_coast_time_threshold)) {
                rocket_state_ = FSMState::STATE_SUSTAINER_IGNITION;
                break;
            }
            if (current_state.acceleration.ax < sustainer_coast_detection_acceleration_threshold) {
                coast_time_ = current_time;
                rocket_state_ = FSMState::STATE_COAST;
            }
            break;
        case FSMState::STATE_COAST:
            if (current_state.velocity.vx <= sustainer_coast_to_apogee_vertical_speed_threshold) {
                apogee_time_ = current_time;
                rocket_state_ = FSMState::STATE_APOGEE;
            }
            break;
        case FSMState::STATE_APOGEE:
            if ((current_state.velocity.vx > sustainer_coast_to_apogee_vertical_speed_threshold) && ((current_time - apogee_time_) < sustainer_apogee_backto_coast_vertical_speed_threshold)) {
                rocket_state_ = FSMState::STATE_COAST;
                break;
            }
            if ((current_time - apogee_time_) > sustainer_apogee_timer_threshold) {
                drogue_time_ = current_time;
                rocket_state_ = FSMState::STATE_DROGUE_DEPLOY;
            }
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            if ((current_time - drogue_time_) < sustainer_pyro_firing_time_minimum) {
                break;
            }
            if ((current_time - drogue_time_) > sustainer_drogue_timer_threshold) {
                rocket_state_ = FSMState::STATE_DROGUE;
            }
            break;
        case FSMState::STATE_DROGUE:
            if ((current_state.position.x <= sustainer_main_deploy_altitude_threshold) && ((current_time - drogue_time_) > main_deploy_delay)) {
                main_time_ = current_time;
                rocket_state_ = FSMState::STATE_MAIN_DEPLOY;
            }
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            if ((current_time - main_time_) < sustainer_pyro_firing_time_minimum) {
                break;
            }
            if ((current_time - main_time_) > sustainer_main_deploy_delay_after_drogue) {
                main_deployed_time_ = current_time;
                rocket_state_ = FSMState::STATE_MAIN;
            }
            break;
        case FSMState::STATE_MAIN:
            if ((std::abs(current_state.velocity.vx) <= sustainer_landed_to_main_vertical_speed_threshold) && ((current_time - main_deployed_time_) > sustainer_landed_vertical_speed_threshold)) {
                landed_time_ = current_time;
                rocket_state_ = FSMState::STATE_LANDED;
            }
            break;
        case FSMState::STATE_LANDED:
            if ((std::abs(current_state.velocity.vx) > sustainer_landed_to_main_vertical_speed_threshold) &&
                ((current_time - landed_time_) > sustainer_landed_time_lockout)) {
                rocket_state_ = FSMState::STATE_MAIN;
            }
            break;
    }
}
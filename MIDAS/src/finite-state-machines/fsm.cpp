#include <cmath>

#include "fsm.h"
#include "thresholds.h"
#include "systems.h"


// helper functions

// most of these helper functions look to average the acceleration values inside the buffer

float FSM::getAcceleration(std::array<HighGData, 8> & hg)
{ 
    float avg = 0;

    for (size_t i = 0; i < hg.size(); i++)
    {
        avg += hg.at(i).gz;
    }

    return (avg / (float)hg.size());
}

float FSM::getAltitude(std::array<Barometer, 8> & bar)
{
    float avg = 0;

    for (size_t i = 0; i < bar.size(); i++)
    {
        avg += bar.at(i).altitude;
    }

    return (avg / (float)bar.size());
}

double FSM::getJerk(std::array<HighGData, 8> & hg, double time_delta)
{

    double avg1 = 0;
    for (size_t i = 0; i < hg.size() / 2; i++)
    {
        avg1 += hg.at(i).gz;
    }
    avg1 = avg1 / hg.size();

    double avg2 = 0;
    for (size_t i = hg.size() / 2; i < hg.size(); i++)
    {
        avg2 += hg.at(i).gz;
    }
    avg2 = avg2 / hg.size();

    return (avg2 - avg1) / (time_delta / 2);
}

double FSM::getVerticalSpeed(std::array<Barometer, 8> & bar, double time_delta)
{
    double avg1 = 0;
    for (size_t i = 0; i < bar.size() / 2; i++)
    {
        avg1 += bar.at(i).altitude;
    }
    avg1 = avg1 / bar.size();

    double avg2 = 0;
    for (size_t i = bar.size() / 2; i < bar.size(); i++)
    {
        avg2 += bar.at(i).altitude;
    }
    avg2 = avg2 / bar.size();

    return (avg2 - avg1) / (time_delta / 2);
}


// code for the fsm update function

FSMState FSM::tick_fsm_sustainer(FSMState & state, std::array<HighGData, 8> & hg, std::array<Barometer, 8> & bar, double time_delta){
    
    //get current time
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());
    
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            // once a significant amount of acceleartion is detected change states
            if(getAcceleration(hg) > sustainer_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            acceleration = getAcceleration(hg); 

            // if acceleration spike was too brief then go back to idle
            if((acceleration < sustainer_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time) < sustainer_idle_to_first_boost_time_threshold)) {
                 state.curr_state = STATE_IDLE;
                 break;
            }

            // once acceleartion decreases to a the threshold go on the next state
            if(acceleration < sustainer_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            // if low acceleration is too brief than go on to the previous state
            if((getAcceleration(hg) >= sustainer_coast_detection_acceleration_threshold) && ((current_time - burnout_time) < sustainer_first_boost_to_burnout_time_threshold)) { 
                state.curr_state = STATE_FIRST_BOOST;
                break;
            }

            // if in burnout for long enough then go on to the next state (time transition)
            if((current_time - burnout_time) > sustainer_first_boost_to_burnout_time_threshold) { 
                sustainer_ignition_time = current_time;
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_SUSTAINER_IGNITION:
            // another time transition into coast after a certain amount of time
            if ((current_time - sustainer_ignition_time) > sustainer_ignition_to_coast_timer_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
                break;
            }

            // once a high enough acceleration is detected then go to next state
            if(getAcceleration(hg) > sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state.curr_state = STATE_SECOND_BOOST;
            }
            
            break;
            
        case STATE_SECOND_BOOST:
            acceleration = getAcceleration(hg); 

            // if high accleration is too brief then return to previous state
            if ((acceleration < sustainer_ignition_to_second_boost_acceleration_threshold) && ((current_time - second_boost_time) < sustainer_ignition_to_second_boost_time_threshold)) { 
                state.curr_state = STATE_SUSTAINER_IGNITION;
                break;
            }

            // if low acceleration detected go to next state
            if (acceleration < sustainer_coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
            }
            break;
            
        case STATE_COAST:
            // if the low acceleration detected was too brief then return to previous state
            if ((getAcceleration(hg) > sustainer_coast_detection_acceleration_threshold) && ((current_time - coast_time) < sustainer_second_boost_to_coast_time_threshold)) {
                state.curr_state = STATE_SECOND_BOOST;
                break;
            }

            // if speed slows down enough then go on to the next stage
            if (getVerticalSpeed(bar, time_delta) <= sustainer_coast_to_apogee_vertical_speed_threshold) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }

            break;
        
        case STATE_APOGEE:

            // if the slow speed was too brief then return to previous state
            if((getVerticalSpeed(bar, time_delta)) > 0 && ((current_time - apogee_time) < sustainer_apogee_check_threshold)) {
                state.curr_state = STATE_COAST;
                break;
            }

            // transition to next state after a certain amount of time
            if((current_time - apogee_time) > sustainer_apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            // if detected a sharp change in jerk then go to next state
            if(getJerk(hg, time_delta) < sustainer_drogue_jerk_threshold) { 
		        state.curr_state = STATE_DROGUE;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if((current_time - drogue_time) > sustainer_drogue_timer_threshold) {
                state.curr_state = STATE_DROGUE;
            }

            break;
        
        case STATE_DROGUE:
            // if altitude low enough then next state
            if(getAltitude(bar) <= sustainer_main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            // if detected a sharp change in jerk then go to the next state
            if(getJerk(hg, time_delta) < sustainer_main_jerk_threshold) { 
		        state.curr_state = STATE_MAIN;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if((current_time - main_time) > sustainer_main_to_main_deploy_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            // if slowed down enough then go on to the next state
            if (abs(getVerticalSpeed(bar, time_delta)) <= sustainer_landed_vertical_speed_threshold) { 
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:
            // if the slow speed was too brief then return to previous state
            if ((abs(getVerticalSpeed(bar, time_delta)) > sustainer_landed_vertical_speed_threshold) && ((current_time - landed_time) > sustainer_landed_timer_threshold)) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
    
    }
    return state;
}

// this is similiar to the previous function but contains less states
FSMState FSM::tick_fsm_booster(FSMState & state, std::array<HighGData, 8> & hg, std::array<Barometer, 8> & bar, double time_delta){

    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            if(getAcceleration(hg) > booster_idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            acceleration = getAcceleration(hg);
            
            if((acceleration < booster_idle_to_first_boost_acceleration_threshold) && ((current_time - launch_time) < booster_idle_to_first_boost_time_threshold)) {
                 state.curr_state = STATE_IDLE;
                 break;
            }
            if(acceleration < booster_coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if((getAcceleration(hg) >= booster_coast_detection_acceleration_threshold) && ((current_time - burnout_time) < booster_first_boost_to_burnout_time_threshold)) { 
                state.curr_state = STATE_FIRST_BOOST;
                break;
            }

            if((current_time - burnout_time) > booster_first_boost_to_burnout_time_threshold) { 
                first_seperation_time = current_time;
                state.curr_state = STATE_FIRST_SEPERATION;
            }
            break;

        case STATE_FIRST_SEPERATION: 
            if (getJerk(hg, time_delta) < booster_first_seperation_jerk_threshold) {
                state.curr_state = STATE_COAST;
                break;
            }

            if((current_time - first_seperation_time) > booster_first_seperation_time_threshold) { 
	            state.curr_state = STATE_COAST;
            }

            break;
            
        case STATE_COAST:
            if (getVerticalSpeed(bar, time_delta) <= 0) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }
            break;
        
        case STATE_APOGEE:
            if(getVerticalSpeed(bar, time_delta) > 0 && ((current_time - apogee_time) < booster_apogee_check_threshold)) {
                state.curr_state = STATE_COAST;
                break;
            }

            if((current_time - apogee_time) > booster_apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            if(getJerk(hg, time_delta) < booster_drogue_jerk_threshold) { 
		        state.curr_state = STATE_DROGUE;
                break;
            }
            if((current_time - drogue_time) > booster_drogue_timer_threshold) {
                state.curr_state = STATE_DROGUE;
            }

            break;
        
        case STATE_DROGUE:
            if(getAltitude(bar) <= booster_main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            if(getJerk(hg, time_delta) < booster_main_jerk_threshold) { 
		        state.curr_state = STATE_MAIN;
                break;
            }

            if((current_time - main_time) > booster_main_to_main_deploy_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            
            if (abs(getVerticalSpeed(bar, time_delta)) <= booster_landed_vertical_speed_threshold) { 
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:

            if ((abs(getVerticalSpeed(bar, time_delta)) > booster_landed_vertical_speed_threshold) && ((current_time - landed_time) > booster_landed_timer_threshold)) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
    
    }
    return state;
}


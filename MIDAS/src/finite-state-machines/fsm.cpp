#include "fsm.h"
#include "thresholds.h"
#include "systems.h"
#include <cmath>
// code for the fsm update function

FSMState tick_fsm_sustainer(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar){
    //pass current state
    double current_time = tick_to_ms(xTaskGetTickCount());
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            if(getAcceleration(hg) > idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            if(getAcceleration(hg) < idle_to_first_boost_acceleration_threshold && current_time - launch_time < idle_to_first_boost_time_threshold) {
                 state.curr_state = STATE_IDLE;
            }
            if(getAcceleration(hg) < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if(getAcceleration(hg) >= coast_detection_acceleration_threshold && (current_time - burnout_time) < first_boost_to_burnout_time_threshold) { 
                state.curr_state = STATE_FIRST_BOOST;
            }

            // Move to next state after a set time?
            // should be 'current_time - burnout_time'
            if(current_time - burnout_time > first_boost_to_burnout_time_threshold) { 
                sustainer_ignition_time = current_time;
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_SUSTAINER_IGNITION:
            if (current_time - sustainer_ignition_time > sustainer_ignition_to_coast_timer_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
            }

            if(getAcceleration(hg) > sustainer_ignition_to_second_boost_acceleration_threshold) {
                second_boost_time = current_time;
                state.curr_state = STATE_SECOND_BOOST;
            }
            
        case STATE_SECOND_BOOST:
            if (getAcceleration(hg) < sustainer_ignition_to_second_boost_acceleration_threshold && (current_time - second_boost_time) < sustainer_ignition_to_second_boost_time_threshold) { 
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            if (getAcceleration(hg) < coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
            }
            break;
            
        case STATE_COAST:
            if (getAcceleration(hg) > coast_detection_acceleration_threshold && current_time - coast_time < second_boost_to_coast_time_threshold) {
                state.curr_state = STATE_SECOND_BOOST;
            }

            if (getVerticalSpeed(bar) <= coast_to_apogee_vertical_speed_threshold) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }

            break;
        
        case STATE_APOGEE:
            if(getVerticalSpeed(bar) > 0 && current_time - apogee_time < apogee_check_threshold) {
                state.curr_state = STATE_COAST;
            }

            if(current_time - apogee_time > apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_DROUGE;
            }
            if(drogue_time > drogue_timer_threshold) {
                state.curr_state = STATE_DROUGE;
            }

            break;
        
        case STATE_DROUGE:
            if(getAltitude(bar) <= main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_MAIN;
            }

            // Move to MAIN if enough time passed since MAIN_DEPLOY
            // Use 'current_time - main_time' right?
            if(current_time - main_time > main_to_main_deploy_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            
            if (abs(getVerticalSpeed(bar)) <= landed_vertical_speed_threshold) { 
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:

            if (abs(getVerticalSpeed(bar)) > landed_vertical_speed_threshold && current_time - landed_time > landed_timer_threshold) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
        
        
        return state;
    
    }
}

FSMState tick_fsm_booster(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar){
    //pass current state
    double current_time = tick_to_ms(xTaskGetTickCount());
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            if(getAcceleration(hg) > idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            if(getAcceleration(hg) < idle_to_first_boost_acceleration_threshold && current_time - launch_time < idle_to_first_boost_time_threshold) {
                 state.curr_state = STATE_IDLE;
            }
            if(getAcceleration(hg) < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if(getAcceleration(hg) >= coast_detection_acceleration_threshold && (current_time - burnout_time) < first_boost_to_burnout_time_threshold) { 
                state.curr_state = STATE_FIRST_BOOST;
            }

            // Move to next state after a set time?
            // should be 'current_time - burnout_time'
            if(current_time - burnout_time > first_boost_to_burnout_time_threshold) { 
                sustainer_ignition_time = current_time;
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_FIRST_SEPERATION: //incomplete
            if (getJerk(hg) < 0) {
                state.curr_state = STATE_COAST;
            }

            first_seperation_time = current_time;

            if(current_time - first_seperation_time > first_seperation_time_threshold) { // move on regardless if it separates or not
	            state.curr_state = STATE_COAST;
                
            }
            
        case STATE_COAST:
            if (getAcceleration(hg) > coast_detection_acceleration_threshold && current_time - coast_time < second_boost_to_coast_time_threshold) {
                state.curr_state = STATE_SECOND_BOOST;
            }

            if (getVerticalSpeed(bar) <= 0) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }
            break;
        
        case STATE_APOGEE:
            if(getVerticalSpeed(bar) > 0 && current_time - apogee_time < apogee_check_threshold) {
                state.curr_state = STATE_COAST;
            }

            if(current_time - apogee_time > apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_DROUGE;
            }
            if(drogue_time > drogue_timer_threshold) {
                state.curr_state = STATE_DROUGE;
            }

            break;
        
        case STATE_DROUGE:
            if(getAltitude(bar) <= main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_MAIN;
            }

            // Move to MAIN if enough time passed since MAIN_DEPLOY
            // Use 'current_time - main_time' right?
            if(current_time - main_time > main_to_main_deploy_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            
            if (abs(getVerticalSpeed(bar)) <= landed_vertical_speed_threshold) { 
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:

            if (abs(getVerticalSpeed(bar)) > landed_vertical_speed_threshold && current_time - landed_time > landed_timer_threshold) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
        
        
        return state;
    
    }
}


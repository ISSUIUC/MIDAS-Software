#include "fsm.h"
#include "thresholds.h"
#include "systems.h"
#include <cmath>

// helper functions

// most of these helper functions look to average the acceleration values inside the buffer

float FSM::getAcceleration(std::array<HighGData, 8> hg)
{ 
    float avg = 0;

    for (size_t i = 0; i < hg.size(); i++)
    {
        avg += (hg.at(i).gz / (float)hg.size());
    }

    return avg;
}

float FSM::getAltitude(std::array<Barometer, 8> bar)
{
    float avg = 0;

    for (size_t i = 0; i < bar.size(); i++)
    {
        avg += (bar.at(i).altitude / (float)bar.size());
    }

    return avg;
}

double FSM::getJerk(std::array<HighGData, 8> hg)
{

    double avg1 = 0;
    for (size_t i = 0; i < hg.size() / 2; i++)
    {
        avg1 += (hg.at(i).gz / hg.size());
    }

    double avg2 = 0;
    for (size_t i = hg.size() / 2; i < hg.size(); i++)
    {
        avg2 += (hg.at(i).gz / hg.size());
    }

    double sec = tick_to_ms(hg.size()) * 1000;

    return (avg2 - avg1) / (sec);
}

double FSM::getVerticalSpeed(std::array<Barometer, 8> bar)
{
    double avg1 = 0;
    for (size_t i = 0; i < bar.size() / 2; i++)
    {
        avg1 += (bar.at(i).altitude / bar.size());
    }

    double avg2 = 0;
    for (size_t i = bar.size() / 2; i < bar.size(); i++)
    {
        avg2 += (bar.at(i).altitude / bar.size());
    }

    double sec = tick_to_ms((unsigned int)bar.size()) * 1000;

    return (avg2 - avg1) / (sec);
}


// code for the fsm update function

FSMState FSM::tick_fsm_sustainer(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar){
    
    //get current time
    double current_time = tick_to_ms(xTaskGetTickCount());
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            // once a significant amount of acceleartion is detected change states
            if(getAcceleration(hg) > idle_to_first_boost_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            // if acceleration spike was too brief then go back to idle
            if(getAcceleration(hg) < idle_to_first_boost_acceleration_threshold && current_time - launch_time < idle_to_first_boost_time_threshold) {
                 state.curr_state = STATE_IDLE;
                 break;
            }

            // once acceleartion decreases to a the threshold go on the next state
            if(getAcceleration(hg) < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            // if low acceleration is too brief than go on to the previous state
            if(getAcceleration(hg) >= coast_detection_acceleration_threshold && (current_time - burnout_time) < first_boost_to_burnout_time_threshold) { 
                state.curr_state = STATE_FIRST_BOOST;
                break;
            }

            // if in burnout for long enough then go on to the next state (time transition)
            if(current_time - burnout_time > first_boost_to_burnout_time_threshold) { 
                sustainer_ignition_time = current_time;
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_SUSTAINER_IGNITION:
            // another time transition into coast after a certain amount of time
            if (current_time - sustainer_ignition_time > sustainer_ignition_to_coast_timer_threshold) {
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
            // if high accleration is too brief then return to previous state
            if (getAcceleration(hg) < sustainer_ignition_to_second_boost_acceleration_threshold && (current_time - second_boost_time) < sustainer_ignition_to_second_boost_time_threshold) { 
                state.curr_state = STATE_SUSTAINER_IGNITION;
                break;
            }

            // if low acceleration detected go to next state
            if (getAcceleration(hg) < coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
            }
            break;
            
        case STATE_COAST:
            // if the low acceleration detected was too brief then retunr to previous state
            if (getAcceleration(hg) > coast_detection_acceleration_threshold && current_time - coast_time < second_boost_to_coast_time_threshold) {
                state.curr_state = STATE_SECOND_BOOST;
                break;
            }

            // if speed slows down enough then go on to the next stage
            if (getVerticalSpeed(bar) <= coast_to_apogee_vertical_speed_threshold) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }

            break;
        
        case STATE_APOGEE:

            // if the slow speed was too brief then return to previous state
            if(getVerticalSpeed(bar) > 0 && current_time - apogee_time < apogee_check_threshold) {
                state.curr_state = STATE_COAST;
                break;
            }

            // transition to next state after a certain amount of time
            if(current_time - apogee_time > apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            // if detected a sharp change in jerk then go to next state
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_DROUGE;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if(drogue_time > drogue_timer_threshold) {
                state.curr_state = STATE_DROUGE;
            }

            break;
        
        case STATE_DROUGE:
            // if altitude low enough then next state
            if(getAltitude(bar) <= main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            // if detected a sharp change in jerk then go to the next state
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_MAIN;
                break;
            }

            // if no transtion after a certain amount of time then just move on to next state
            if(current_time - main_time > main_to_main_deploy_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            // if slowed down enough then go on to the next state
            if (abs(getVerticalSpeed(bar)) <= landed_vertical_speed_threshold) { 
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:
            // if the slow speed was too brief then return to previous state
            if (abs(getVerticalSpeed(bar)) > landed_vertical_speed_threshold && current_time - landed_time > landed_timer_threshold) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
        
        default:
            return state;
    
    }
    return state;
}

// this is similiar to the previous function but contains less states
FSMState FSM::tick_fsm_booster(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar){

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
                 break;
            }
            if(getAcceleration(hg) < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if(getAcceleration(hg) >= coast_detection_acceleration_threshold && (current_time - burnout_time) < first_boost_to_burnout_time_threshold) { 
                state.curr_state = STATE_FIRST_BOOST;
                break;
            }

            if(current_time - burnout_time > first_boost_to_burnout_time_threshold) { 
                sustainer_ignition_time = current_time;
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_FIRST_SEPERATION: 
            if (getJerk(hg) < 0) {
                state.curr_state = STATE_COAST;
                break;
            }

            first_seperation_time = current_time;

            if(current_time - first_seperation_time > first_seperation_time_threshold) { 
	            state.curr_state = STATE_COAST;
            }

            break;
            
        case STATE_COAST:
            if (getAcceleration(hg) > coast_detection_acceleration_threshold && current_time - coast_time < second_boost_to_coast_time_threshold) {
                state.curr_state = STATE_SECOND_BOOST;
                break;
            }

            if (getVerticalSpeed(bar) <= 0) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }
            break;
        
        case STATE_APOGEE:
            if(getVerticalSpeed(bar) > 0 && current_time - apogee_time < apogee_check_threshold) {
                state.curr_state = STATE_COAST;
                break;
            }

            if(current_time - apogee_time > apogee_timer_threshold) {
                drogue_time = current_time;
                state.curr_state = STATE_DROGUE_DEPLOY;
            }
            break;
            
        case STATE_DROGUE_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_DROUGE;
                break;
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
                break;
            }

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
        
        
        default:
            return state;

    
    }
    return state;
}


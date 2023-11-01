#include "fsm.h"
#include "thresholds.h"
#include "systems.h"

// code for the fsm update function

FSMState tick_fsm(FSMState state, std::array<HighGData, 8> hg, std::array<Barometer, 8> bar){
    //pass current state
    double current_time = tick_to_ms(xTaskGetTickCount());
    
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            if(getAcceleration(hg) > launch_detection_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            if(getAcceleration(hg) < launch_detection_acceleration_threshold && current_time - launch_time < idle_to_first_boost_time_threshold) {
                 state.curr_state = STATE_IDLE;
            }
            if(getAcceleration(hg) < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if(getAcceleration(hg) >= coast_detection_acceleration_threshold && (current_time - burnout_time) < 1000) {
                state.curr_state = STATE_FIRST_BOOST;
            }

            if(burnout_time > 3000) {
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_SUSTAINER_IGNITION:
            if(getAcceleration(hg) > first_separation_linear_acceleration_thresh) {
                second_boost_time = current_time;
                state.curr_state = STATE_SECOND_BOOST;
            }
            
        case STATE_SECOND_BOOST:
            if (getAcceleration(hg) < first_separation_linear_acceleration_thresh && (current_time - second_boost_time) < 1) {
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

            if (getVerticalSpeed(bar) <= 0) {
             	apogee_time = current_time;
             	state.curr_state = STATE_APOGEE; 
            }

            break;
        
        case STATE_APOGEE:
            if(getVerticalSpeed(bar) > 0 && current_time - apogee_time < apogee_check_threshold) {
                state.curr_state = STATE_COAST;
            }

            if(apogee_time > apogee_timer_threshold) {
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
            double avg1 = 0;
            for (size_t i = 0; i < bar.size()/2; i++)
            {
                avg1 += (bar.at(i).altitude/bar.size());
            }
            if(avg1 <= main_deploy_altitude_threshold) {
                state.curr_state = STATE_MAIN_DEPLOY;
                main_time = current_time;
            }
            break;

        case STATE_MAIN_DEPLOY:
            if(getJerk(hg) < 0) { 
		        state.curr_state = STATE_MAIN;
            }

            if(main_time > main_timer_threshold) {
                state.curr_state = STATE_MAIN;
            }
            break;

        case STATE_MAIN:
            
            if (getVerticalSpeed(bar) <= 0) {
             	landed_time = current_time;
             	state.curr_state = STATE_LANDED; 
            }
            break;

        case STATE_LANDED:

            if (getVerticalSpeed(bar) > 0 && current_time - landed_time > landed_timer_threshold) {
             	state.curr_state = STATE_MAIN; 
            }
            break;
        
        
        return state;
    
    }
}
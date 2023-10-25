#include "fsm.h"
#include "thresholds.h"
#include "systems.h"

// code for the fsm update function

FSMState tick_fsm(FSMState state, HighGData hg){
    //pass current state
    double current_time = tick_to_ms(xTaskGetTickCount());
    
    
    switch (state.curr_state)
    {
        case STATE_IDLE:
            if(hg.gz > launch_detection_acceleration_threshold) {
                launch_time = current_time;
                state.curr_state = STATE_FIRST_BOOST;
            }
            
            break;

        case STATE_FIRST_BOOST:
            if(hg.gz < launch_detection_acceleration_threshold && current_time - launch_time < idle_to_first_boost_time_threshold) {
                 state.curr_state = STATE_IDLE;
            }
            if(hg.gz < coast_detection_acceleration_threshold) {
                burnout_time = current_time;
                state.curr_state  = STATE_BURNOUT;
            }

            break;

        case STATE_BURNOUT:
            if(hg.gz >= coast_detection_acceleration_threshold && (current_time - burnout_time) < 1000) {
                state.curr_state = STATE_FIRST_BOOST;
            }

            if(burnout_time > 3000) {
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            break;

        case STATE_SUSTAINER_IGNITION:
            if(hg.gz > first_separation_linear_acceleration_thresh) {
                second_boost_time = current_time;
                state.curr_state = STATE_SECOND_BOOST;
            }
            
        case STATE_SECOND_BOOST:
            if (hg.gz < first_separation_linear_acceleration_thresh && (current_time - second_boost_time) < 1) {
                state.curr_state = STATE_SUSTAINER_IGNITION;
            }
            if (hg.gz < coast_detection_acceleration_threshold) {
                coast_time = current_time;
                state.curr_state = STATE_COAST;
            }
            break;
            
        case STATE_COAST:
            if (hg.gz > coast_detection_acceleration_threshold && current_time - coast_time < second_boost_to_coast_time_threshold) {
                state.curr_state = STATE_SECOND_BOOST;
            }

            // NEED TO ADD WITH BUFFER:
                // if(change_in_altitude <= apogee_threshold) {
                // 	apogee_time = current_time;
                // 	rocket_state_ = STATE_APOGEE; 
                // }

            break;
        
        case STATE_APOGEE:
            if() {
                
            }
        case STATE_DROGUE_DEPLOY:
            // NEED TO ADD WITH BUFFER
            // if(jerk < 0) { 
		        //rocket_state_ = FSM_State::STATE_DROUGE;
            //}
            if(drogue_time > )

            break;
        
        case STATE_DROUGE:
        
            break;
        
        return state;


    
    }
}
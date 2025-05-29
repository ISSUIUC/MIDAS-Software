#pragma once

/**
 * @enum FSMState
 * 
 * @brief Enum for the different FSM states
*/

enum FSMState {
    STATE_IDLE, //Cameras are off
    STATE_ON,   //Cameras are recording
    STATE_RECORDING_ASCENT,     //Could be combined with STATE_ON
    STATE_RECORDING_DESCENT,    //Switches transmitting camera
    FSM_STATE_COUNT                 //used to get the total number of fsm states so we can assert that the fsm will fit in 3 bits
};
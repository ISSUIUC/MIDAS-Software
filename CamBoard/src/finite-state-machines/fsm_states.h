#pragma once

/**
 * @enum FSMState
 * 
 * @brief Enum for the different FSM states
*/

// enum FSMState {
//     STATE_IDLE,
//     STATE_FIRST_BOOST,
//     STATE_BURNOUT,
//     STATE_COAST,
//     STATE_APOGEE,
//     STATE_DROGUE_DEPLOY,
//     STATE_DROGUE,
//     STATE_MAIN_DEPLOY,
//     STATE_MAIN,
//     STATE_LANDED,
//     STATE_SUSTAINER_IGNITION,
//     STATE_SECOND_BOOST,
//     STATE_FIRST_SEPARATION,
//     FSM_STATE_COUNT                 //used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
// };

enum FSMState {
    STATE_IDLE,
    STATE_ON,
    STATE_RECORDING_ASCENT,
    STATE_RECORDING_DESCENT,
    FSM_STATE_COUNT                 //used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
};
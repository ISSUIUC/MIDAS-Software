#pragma once

/**
 * @enum FSMState
 * 
 * @brief Enum for the different FSM states
*/

enum FSMState {
    STATE_SAFE,                     // SAFE state
    STATE_PYRO_TEST,                // Unsafe pyro testing state (accepts pyro firing commands)
    STATE_IDLE,                     // State when the rocket is on pad
    STATE_FIRST_BOOST,              // State when the rocket detects successful firing of the booster motor
    STATE_BURNOUT,                  
    STATE_COAST,                    
    STATE_APOGEE,                   // Delay state before firing APOGEE
    STATE_DROGUE_DEPLOY,            // State triggered to fire the APOGEE channel
    STATE_DROGUE,                   // State for drogue-based descent
    STATE_MAIN_DEPLOY,              // State triggered to fire the MAIN pyro channel
    STATE_MAIN,                     // State when the rocket is descending on the MAIN chute
    STATE_LANDED,                   // State triggered after landing has been detected
    STATE_SUSTAINER_IGNITION,       // State triggered to fire the MOTOR channel to ignite sustainer
    STATE_SECOND_BOOST,             // State when the rocket detects successful firing of the sustainer motor
    STATE_FIRST_SEPARATION,         // 
    FSM_STATE_COUNT                 //used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
};

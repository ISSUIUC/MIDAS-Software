#pragma once

/**
 * @enum FSMState
 * 
 * @brief Enum for the different FSM states
*/

enum FSMState {
    STATE_SAFE,                     // (a) SAFE state
    STATE_PYRO_TEST,                // (b) Unsafe pyro testing state (accepts pyro firing commands)
    STATE_IDLE,                     // (c) State when the rocket is on pad
    STATE_FIRST_BOOST,              // (d) State when the rocket detects successful firing of the booster motor
    STATE_BURNOUT,                  // (e) 
    STATE_COAST,                    // (f)
    STATE_APOGEE,                   // (g) Delay state before firing APOGEE
    STATE_DROGUE_DEPLOY,            // (h) State triggered to fire the APOGEE channel
    STATE_DROGUE,                   // (i) State for drogue-based descent
    STATE_MAIN_DEPLOY,              // (j) State triggered to fire the MAIN pyro channel
    STATE_MAIN,                     // (k) State when the rocket is descending on the MAIN chute
    STATE_LANDED,                   // (l) State triggered after landing has been detected
    STATE_SUSTAINER_IGNITION,       // (m) State triggered to fire the MOTOR channel to ignite sustainer
    STATE_SECOND_BOOST,             // (n) State when the rocket detects successful firing of the sustainer motor
    STATE_FIRST_SEPARATION,         // (o)
    FSM_STATE_COUNT                 //used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
};

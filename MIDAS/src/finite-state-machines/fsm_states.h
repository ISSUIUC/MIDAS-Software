#pragma once

// holds the enum for different FSM states

enum FSMState {
    STATE_IDLE,
    STATE_FIRST_BOOST,
    STATE_BURNOUT,
    STATE_COAST,
    STATE_APOGEE,
    STATE_DROGUE_DEPLOY,
    STATE_DROGUE,
    STATE_MAIN_DEPLOY,
    STATE_MAIN,
    STATE_LANDED,
// #ifdef IS_SUSTAINER
    STATE_SUSTAINER_IGNITION,
    STATE_SECOND_BOOST,
// #else
    STATE_FIRST_SEPARATION,
// #endif
    FSM_STATE_COUNT, //used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
};

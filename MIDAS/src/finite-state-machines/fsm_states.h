#pragma once

// holds the enum for different FSM states

enum class FSMState {
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
};

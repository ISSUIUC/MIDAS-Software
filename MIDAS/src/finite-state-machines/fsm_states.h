#pragma once

// holds the enum for different FSM states

enum FSM_state{
    STATE_IDLE,
    STATE_FIRST_BOOST, 
    STATE_BURNOUT, 
    STATE_SUSTAINER_IGNITION, 
    STATE_SECOND_BOOST, 
    STATE_COAST, 
    STATE_APOGEE, 
    STATE_DROGUE_DEPLOY, 
    STATE_DROUGE, 
    STATE_MAIN_DEPLOY, 
    STATE_MAIN,
    STATE_LANDED,
};
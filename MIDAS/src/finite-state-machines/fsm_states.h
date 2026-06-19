#pragma once

#include <cstdint>

/**
 * @enum FSMState
 *
 * @brief Enum for the different FSM states
*/

enum FSMState {
    STATE_SAFE,                     // SAFE state
    STATE_PYRO_TEST,                // Unsafe pyro testing state (accepts pyro firing commands)
    STATE_ARMED,                    // State when the rocket is on pad and able to detect launch
    STATE_BOOST,                    // State when the rocket is under acceleration from a motor
    STATE_COAST,                    // State when the rocket is coasting between or after motor burns
    STATE_DROGUE,                   // State when the rocket is descending under drogue chute after apogee
    STATE_MAIN,                     // State when the rocket is descending under main chute after a set altitude
    STATE_LANDED,                   // State when the rocket is landed
    FSM_STATE_COUNT                 // used to get the total number of fsm states so we can assert that the fsm will fit in 4 bits
};

struct FSMData {
    FSMState state;
    uint8_t current_motor;
};

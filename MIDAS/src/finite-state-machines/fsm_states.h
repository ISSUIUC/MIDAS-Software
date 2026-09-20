#pragma once

#include <cstdint>

/**
 * @enum FSMState
 *
 * @brief Enumerates all possible finite state machine (FSM) flight states.
 *
 * These states represent the major phases of flight, from pre-launch
 * through landing. The FSM transitions between these states based on
 * sensor data and configured flight logic.
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

/**
 * @struct FSMData
 *
 * @brief Stores the current state of the finite state machine.
 *
 * This structure tracks the active flight state along with the current
 * motor or stage index, allowing other subsystems to monitor the rocket's
 * flight progress.
 */
struct FSMData {

    /// Current finite state machine state.
    FSMState state;

    /// Index of the currently active or most recently completed motor.
    uint8_t current_motor;
};

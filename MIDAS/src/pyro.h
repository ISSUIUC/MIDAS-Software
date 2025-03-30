#pragma once

#include <cmath>

#include "sensor_data.h"
#include "rocket_state.h"
#include "hal.h"


template<typename Hw>
struct PyroLogic {
private:
    HwInterface<Hw>& hw;
    double safety_pyro_start_firing_time;    // Time when pyros have fired "this cycle" (pyro test) -- Used to only fire pyros for a time then transition to SAFE
    bool safety_has_fired_pyros_this_cycle;  // If pyros have fired "this cycle" (pyro test) -- Allows only firing 1 pyro per cycle.

public:
    explicit PyroLogic(HwInterface<Hw>& hw) : hw(hw) { }
    PyroState tick(FSMState fsm_state, OrientationData orientation, CommandFlags& telem_commands);

    void disarm_all_channels(PyroState& prev_state) {
        hw.set_global_arm(false);
        hw.set_pin_firing(Channel::A, false);
        hw.set_pin_firing(Channel::B, false);
        hw.set_pin_firing(Channel::C, false);
        hw.set_pin_firing(Channel::D, false);

        prev_state.is_global_armed = false;

        for (size_t i = 0; i < 4; ++i) {
            // Update each channel's state sequentially
            prev_state.channel_firing[i] = false;
        }
    }

    void set_pyro_safety() {
        safety_pyro_start_firing_time = pdTICKS_TO_MS(xTaskGetTickCount());
        safety_has_fired_pyros_this_cycle = true;
    }

    void reset_pyro_safety() {
        safety_has_fired_pyros_this_cycle = false;
    }
};

inline bool can_fire_igniter(OrientationData orientation) {
    // With new GNC orientation code we can add a simple check.
    return orientation.tilt < (M_PI/5);    // 36 degrees;
}

#define PYRO_TEST_FIRE_TIME 100

#ifdef IS_SUSTAINER

/**
 * @brief Upper stage only! Fires channels by setting their pin on the GPIO.
 *
 * @return A pyro struct indicating which pyro channels are armed and/or firing.
 */
template<typename Hw>
PyroState PyroLogic<Hw>::tick(FSMState fsm_state, OrientationData orientation, CommandFlags& telem_commands) {
    PyroState new_pyro_state = PyroState();
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (fsm_state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // If the state is not SAFE, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    hw.set_global_arm(true);

    switch (fsm_state) {
        case FSMState::STATE_IDLE: {
            reset_pyro_safety(); // Ensure that pyros can be fired when we transition away from this state
            break;
        }
        case FSMState::STATE_PYRO_TEST: {
            if (safety_has_fired_pyros_this_cycle) {
                // If a fire pyro command has already been acknowledged, do not acknowledge more commands, just fire pyro for the defined time
                // then, transition to SAFE.
                if ((current_time - safety_pyro_start_firing_time) >= PYRO_TEST_FIRE_TIME) {
                    telem_commands.should_transition_safe = true;
                    disarm_all_channels(new_pyro_state);
                    telem_commands.should_fire_pyro_a = false;
                    telem_commands.should_fire_pyro_b = false;
                    telem_commands.should_fire_pyro_c = false;
                    telem_commands.should_fire_pyro_d = false;

                    reset_pyro_safety();
                }
                break;
            }

            // Respond to telem commands to fire igniters
            if (telem_commands.should_fire_pyro_a) {
                // Fire pyro channel "A"
                new_pyro_state.channel_firing[0] = true;
                hw.set_pin_firing(Channel::A, true);
                set_pyro_safety();
            }

            if (telem_commands.should_fire_pyro_b) {
                // Fire pyro channel "B"
                new_pyro_state.channel_firing[1] = true;
                hw.set_pin_firing(Channel::B, true);
                set_pyro_safety();
            }

            if (telem_commands.should_fire_pyro_c) {
                // Fire pyro channel "C"
                new_pyro_state.channel_firing[2] = true;
                hw.set_pin_firing(Channel::C, true);
                set_pyro_safety();
            }

            if (telem_commands.should_fire_pyro_d) {
                // Fire pyro channel "D"
                new_pyro_state.channel_firing[0] = true;
                hw.set_pin_firing(Channel::D, true);
                set_pyro_safety();
            }

            break;
        }
        case FSMState::STATE_SUSTAINER_IGNITION: {
            // Fire "Pyro C" to ignite sustainer (Pyro C is motor channel)
            // Additionally, check if orientation allows for firing
            if (can_fire_igniter(orientation)) {
                new_pyro_state.channel_firing[2] = true;
                hw.set_pin_firing(Channel::C, true);
            }
            break;
        }
        case FSMState::STATE_DROGUE_DEPLOY: {
            // Fire "Pyro A" to deploy upper stage drogue
            new_pyro_state.channel_firing[0] = true;
            hw.set_pin_firing(Channel::A, true);
            break;
        }
        case FSMState::STATE_MAIN_DEPLOY: {
            // Fire "Pyro B" to deploy main.
            new_pyro_state.channel_firing[1] = true;
            hw.set_pin_firing(Channel::B, true);
            break;
        }
        default:
            break;
    }

    return new_pyro_state;
}

#endif

#ifdef IS_BOOSTER

/**
 * Lower stage only!
 *
 * @brief Fires channels by setting their pin on the GPIO.
 *
 * @return A new pyro struct, with data depending on whether or not each pyro channel should be firing.
*/
template<typename Hw>
PyroState PyroLogic<Hw>::tick(FSMState fsm_state, OrientationData orientation, CommandFlags& telem_commands) {
    PyroState new_pyro_state = PyroState();
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (fsm_state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // If the state is not SAFE, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    hw.set_global_arm(true);
    // If the state is IDLE or any state after that, we arm the global arm pin
    switch (fsm_state) {
        case FSMState::STATE_IDLE:
            reset_pyro_safety(); // Ensure that pyros can be fired when we transition away from this state
            break;
        case FSMState::STATE_PYRO_TEST:

            if(safety_has_fired_pyros_this_cycle) {
                // If a fire pyro command has already be acknowledged, do not acknowledge more commands, just fire pyro for the defined time
                // then, transition to SAFE.
                if((current_time - safety_pyro_start_firing_time) >= PYRO_TEST_FIRE_TIME) {
                    telem_commands.should_transition_safe = true;
                    disarm_all_channels(new_pyro_state);
                    telem_commands.should_fire_pyro_a = false;
                    telem_commands.should_fire_pyro_b = false;
                    telem_commands.should_fire_pyro_c = false;
                    telem_commands.should_fire_pyro_d = false;

                    reset_pyro_safety();
                }
                break;
            }

            // Respond to telem commands to fire igniters
            if(telem_commands.should_fire_pyro_a) {
                // Fire pyro channel "A"
                new_pyro_state.channel_firing[0] = true;
                hw.set_pin_firing(Channel::A, true);
                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_b) {
                // Fire pyro channel "B"
                new_pyro_state.channel_firing[1] = true;
                hw.set_pin_firing(Channel::B, true);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_c) {
                // Fire pyro channel "C"
                new_pyro_state.channel_firing[2] = true;
                hw.set_pin_firing(Channel::C, true);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_d) {
                // Fire pyro channel "D"
                new_pyro_state.channel_firing[3] = true;
                hw.set_pin_firing(Channel::D, true);

                set_pyro_safety();
            }

            break;
        case FSMState::STATE_FIRST_SEPARATION:
            // Fire "Pyro D" when separating stage 1
            new_pyro_state.channel_firing[3] = true;
            hw.set_pin_firing(Channel::D, true);
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            // Fire "Pyro A" to deploy drogue
            new_pyro_state.channel_firing[0] = true;
            hw.set_pin_firing(Channel::A, true);
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            // Fire "Pyro B" to deploy Main
            new_pyro_state.channel_firing[1] = true;
            hw.set_pin_firing(Channel::B, true);
            break;
        default:
            break;
    }

    return new_pyro_state;
}

#endif

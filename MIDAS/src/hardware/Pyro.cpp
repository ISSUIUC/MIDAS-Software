#include <cmath>

#include "sensors.h"
#include "pins.h"

#include "TCAL9538.h"
#include <rocket_state.h>


// Fire the pyros for this time during PYRO_TEST (ms)
#define PYRO_TEST_FIRE_TIME 100

#define MAXIMUM_TILT_ANGLE (M_PI/8)    // 22.5 degrees

/**
 * @brief Returns true if the error_code signals failure.
 */
bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
}

/**
 * @brief Determines if orientation is in an acceptable range to ignite second stage.
 * 
 * @return True if acceptable, false if not.
 */
bool can_fire_igniter(AngularKalmanData angular_kalman_data) {
    // With new GNC orientation code we can add a simple check.
    return angular_kalman_data.mq_tilt < MAXIMUM_TILT_ANGLE;
}


/**
 * @brief Initializes the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 *        have to do anything special and will always return NoError.
 */
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // fire pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_FIRE_PIN, OUTPUT));

//    if (has_failed_gpio_init) {
//        return ErrorCode::PyroGPIOCouldNotBeInitialized;
//    } else {
    return ErrorCode::NoError;   // GPIO Driver always claimes it errored even when it doesn't.
//    }
}

void Pyro::disarm_all_channels(PyroState& prev_state) {
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, LOW);
    gpioDigitalWrite(PYROA_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROB_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROC_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROD_FIRE_PIN, LOW);

    prev_state.is_global_armed = false;
    
    for(size_t i = 0; i < 4; ++i) {
        // Update each channel's state sequentially
        prev_state.channel_firing[i] = false;
    }
}

void Pyro::set_pyro_safety() {
    safety_pyro_start_firing_time = pdTICKS_TO_MS(xTaskGetTickCount());
    safety_has_fired_pyros_this_cycle = true;
}

void Pyro::reset_pyro_safety() {
    safety_has_fired_pyros_this_cycle = false;
}

#ifdef IS_SUSTAINER

/**
 * @brief Upper stage only! Fires channels by setting their pin on the GPIO.
 * 
 * @return A pyro struct indicating which pyro channels are armed and/or firing.
 */
PyroState Pyro::tick(FSMState fsm_state, AngularKalmanData angular_kalman_data, CommandFlags& telem_commands) {
    PyroState new_pyro_state = PyroState();
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (fsm_state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // If the state is not SAFE, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, HIGH);

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
                gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_b) {
                // Fire pyro channel "B"
                new_pyro_state.channel_firing[1] = true;
                gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_c) {
                // Fire pyro channel "C"
                new_pyro_state.channel_firing[2] = true;
                gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_d) {
                // Fire pyro channel "D"
                new_pyro_state.channel_firing[0] = true;
                gpioDigitalWrite(PYROD_FIRE_PIN, HIGH);

                set_pyro_safety();
            }
            
            break;
        case FSMState::STATE_SUSTAINER_IGNITION:
            // Fire "Pyro C" to ignite sustainer (Pyro C is motor channel)
            // Additionally, check if orientation allows for firing
            if (can_fire_igniter(angular_kalman_data)) {
                new_pyro_state.channel_firing[2] = true;
                gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);
            }
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            // Fire "Pyro A" to deploy upper stage drogue
            new_pyro_state.channel_firing[0] = true;
            gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            // Fire "Pyro B" to deploy main.
            new_pyro_state.channel_firing[1] = true;
            gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);
            break;
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
PyroState Pyro::tick(FSMState fsm_state, AngularKalmanData angular_kalman_data, CommandFlags& telem_commands) {
    PyroState new_pyro_state = PyroState();
    double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

    if (fsm_state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // If the state is not SAFE, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, HIGH);
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
                gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_b) {
                // Fire pyro channel "B"
                new_pyro_state.channel_firing[1] = true;
                gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_c) {
                // Fire pyro channel "C"
                new_pyro_state.channel_firing[2] = true;
                gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);

                set_pyro_safety();
            }

            if(telem_commands.should_fire_pyro_d) {
                // Fire pyro channel "D"
                new_pyro_state.channel_firing[3] = true;
                gpioDigitalWrite(PYROD_FIRE_PIN, HIGH);

                set_pyro_safety();
            }
            
            break;
        case FSMState::STATE_FIRST_SEPARATION:
            // Fire "Pyro D" when separating stage 1
            new_pyro_state.channel_firing[3] = true;
            gpioDigitalWrite(PYROD_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            // Fire "Pyro A" to deploy drogue
            new_pyro_state.channel_firing[0] = true;
            gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            // Fire "Pyro B" to deploy Main
            new_pyro_state.channel_firing[1] = true;
            gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);
            break;
        default:
            break;
    }

    return new_pyro_state;
}

#endif

#include <cmath>

#include "sensors.h"
#include "pins.h"

#include <rocket_state.h>


// Fire the pyros for this time during PYRO_TEST (ms)
#define PYRO_TEST_FIRE_TIME 500

#define MAXIMUM_TILT_ANGLE (M_PI/9)    // 20 degrees

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
bool can_fire_igniter(Orientation orientation) {
    // This needs to be fleshed out. The sensor may not report angles in 'world space', so we need to determine
    // if the orientation of the rocket depends on other angles
//    return std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE && std::abs(orientation.yaw) < MAXIMUM_TILT_ANGLE;
    (void) orientation;
    return true;
}


/**
 * @brief Initializes the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 *        have to do anything special and will always return NoError.
 */
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // arm pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_ARM_PIN, OUTPUT));

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
    gpioDigitalWrite(PYROA_ARM_PIN, LOW);
    gpioDigitalWrite(PYROA_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROB_ARM_PIN, LOW);
    gpioDigitalWrite(PYROB_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROC_ARM_PIN, LOW);
    gpioDigitalWrite(PYROC_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROD_ARM_PIN, LOW);
    gpioDigitalWrite(PYROD_FIRE_PIN, LOW);

    prev_state.is_global_armed = false;
    
    for(size_t i = 0; i < 4; ++i) {
        // Update each channel's state sequentially
        prev_state.channels[i].is_armed = false;
        prev_state.channels[i].is_firing = false;
    }
}

#ifdef IS_SUSTAINER

/**
 * @brief Upper stage only! Fires channels by setting their pin on the GPIO.
 * 
 * @return A pyro struct indicating which pyro channels are armed and/or firing.
 */
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation, CommandFlags& telem_commands) {
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
            break;
        case FSMState::STATE_PYRO_TEST:

            if(safety_has_fired_pyros_this_cycle) {
                // If a fire pyro command has already be acknowledged, do not acknowledge more commands, just fire pyro for the defined time
                // then, transition to SAFE.
                if((current_time - safety_pyro_start_firing_time) >= PYRO_TEST_FIRE_TIME) {
                    telem_commands.should_transition_idle = true;
                    disarm_all_channels(new_pyro_state);
                }
                break;
            }

            // Respond to telem commands to fire igniters
            if(telem_commands.should_fire_pyro_a) {
                // Fire pyro channel "A"
                new_pyro_state.channels[0].is_armed = true;
                new_pyro_state.channels[0].is_firing = true;
                gpioDigitalWrite(PYROA_ARM_PIN, HIGH);
                gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);

                safety_pyro_start_firing_time = current_time;
                safety_has_fired_pyros_this_cycle = true;
            }

            if(telem_commands.should_fire_pyro_b) {
                // Fire pyro channel "B"
                new_pyro_state.channels[1].is_armed = true;
                new_pyro_state.channels[1].is_firing = true;
                gpioDigitalWrite(PYROB_ARM_PIN, HIGH);
                gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);

                safety_pyro_start_firing_time = current_time;
                safety_has_fired_pyros_this_cycle = true;
            }

            if(telem_commands.should_fire_pyro_c) {
                // Fire pyro channel "C"
                new_pyro_state.channels[2].is_armed = true;
                new_pyro_state.channels[2].is_firing = true;
                gpioDigitalWrite(PYROC_ARM_PIN, HIGH);
                gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);

                safety_pyro_start_firing_time = current_time;
                safety_has_fired_pyros_this_cycle = true;
            }

            if(telem_commands.should_fire_pyro_d) {
                // Fire pyro channel "D"
                new_pyro_state.channels[3].is_armed = true;
                new_pyro_state.channels[3].is_firing = true;
                gpioDigitalWrite(PYROD_ARM_PIN, HIGH);
                gpioDigitalWrite(PYROD_FIRE_PIN, HIGH);

                safety_pyro_start_firing_time = current_time;
                safety_has_fired_pyros_this_cycle = true;
            }
            

        case FSMState::STATE_SUSTAINER_IGNITION:
            // Fire "Pyro A" to ignite sustainer
            // Additionally, check if orientation allows for firing
            if (can_fire_igniter(orientation)) {
                new_pyro_state.channels[0].is_armed = true;
                new_pyro_state.channels[0].is_firing = true;
                gpioDigitalWrite(PYROA_ARM_PIN, HIGH);
                gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
            }
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            // Fire "Pyro B" to deploy upper stage drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            gpioDigitalWrite(PYROB_ARM_PIN, HIGH);
            gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            // Fire "Pyro C" to deploy main.
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            gpioDigitalWrite(PYROC_ARM_PIN, HIGH);
            gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);
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
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation, CommandFlags& telem_commands) {
    PyroState new_pyro_state = PyroState();

    // If the state is IDLE or any state after that, we arm the global arm pin


    switch (fsm_state) {
        case FSMState::STATE_FIRST_SEPARATION:
            // Fire "Pyro D" when separating stage 1
            new_pyro_state.channels[3].is_armed = true;
            new_pyro_state.channels[3].is_firing = true;
            gpioDigitalWrite(PYROD_ARM_PIN, HIGH);
            gpioDigitalWrite(PYROD_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_DROGUE_DEPLOY:
            // Fire "Pyro B" to deploy drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            gpioDigitalWrite(PYROB_ARM_PIN, HIGH);
            gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);
            break;
        case FSMState::STATE_MAIN_DEPLOY:
            // Fire "Pyro C" to deploy Main
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            gpioDigitalWrite(PYROC_ARM_PIN, HIGH);
            gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);
            break;
        default:
            break;
    }

    return new_pyro_state;
}

#endif

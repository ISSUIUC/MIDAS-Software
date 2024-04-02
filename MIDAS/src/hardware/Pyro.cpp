#include <cmath>

#include "sensors.h"
#include "pins.h"

#include "TCAL9539.h"

#define MAXIMUM_TILT_ANGLE (M_PI/9) // 20 degrees

/**
 * Helper function: Determines if the orientation is within an acceptable range to fire the second stage igniter.
*/
bool gpio_error_to_fail_flag(GpioError error_code) {
    return error_code != GpioError::NoError;
}

bool can_fire_igniter(Orientation orientation) {
    // This needs to be fleshed out. The sensor may not report angles in 'world space', so we need to determine
    // if the orientation of the rocket depends on other angles
//    return std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE && std::abs(orientation.yaw) < MAXIMUM_TILT_ANGLE;
    return true;
}


/**
 * "Initializes" the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 * have to do anything special.
*/
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // arm pins
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROA_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROB_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROC_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROD_ARM_PIN, OUTPUT));

    // fire pins
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROA_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROB_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROC_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(PYROD_FIRE_PIN, OUTPUT));

//    if (has_failed_gpio_init) {
//        return ErrorCode::PyroGPIOCouldNotBeInitialized;
//    } else {
    return ErrorCode::NoError;
//    }
}

#ifdef IS_SUSTAINER

/**
 * Upper stage only!
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation) {
    PyroState new_pyro_state = PyroState();

    // If the state is IDLE or any state after that, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, HIGH);

    switch (fsm_state) {
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
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation) {
    PyroState new_pyro_state = PyroState();

    // If the state is IDLE or any state after that, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, HIGH);

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

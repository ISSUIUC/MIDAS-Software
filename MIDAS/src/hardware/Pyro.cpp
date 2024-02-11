#include "sensors.h"
#include "pins.h"
#include <cmath>

#include "TCAL9539.h"

#define MAXIMUM_TILT_ANGLE M_PI/9 // 20 degrees
#define GPIO_ID 0

#define IS_UPPER_STAGE // Only one of these should be defined!
// #define IS_LOWER_STAGE

/**
 * Helper function: Determines if the orientation is within an acceptable range to fire the second stage igniter.
*/
bool gpio_error_to_fail_flag(GpioError error_code) {
    return error_code != GpioError::NoError;
}

bool can_fire_igniter(Orientation orientation) {
    // This needs to be fleshed out. The sensor may not report angles in 'world space', so we need to determine
    // if the orientation of the rocket depends on other angles
    return std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE && std::abs(orientation.yaw) < MAXIMUM_TILT_ANGLE;
}


/**
 * "Initializes" the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 * have to do anything special.
*/
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), OUTPUT));

    // arm pins
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROA_ARM_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROB_ARM_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROC_ARM_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROD_ARM_PIN), OUTPUT));

    // fire pins
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROC_ARM_PIN), OUTPUT));
    has_failed_gpio_init |= gpio_error_to_fail_flag(gpioPinMode(GpioAddress(GPIO_ID, PYROD_FIRE_PIN), OUTPUT));

    if (has_failed_gpio_init) {
        return ErrorCode::PyroGPIOCouldNotBeInitialized;
    } else {
        return ErrorCode::NoError;
    }
    
}

#ifdef IS_UPPER_STAGE

/**
 * Upper stage only!
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation) {
    PyroState new_pyro_state = PyroState();

    // If the state is IDLE or any state after that, we arm the global arm pin
    if(fsm_state.curr_state >= FSM_state::STATE_IDLE) {
        new_pyro_state.is_global_armed = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), HIGH);
    }

    switch (fsm_state.curr_state) {
        case FSM_state::STATE_SUSTAINER_IGNITION:
            // Fire "Pyro A" to ignite sustainer
            // Additionally, check if orientation allows for firing
            if (can_fire_igniter(orientation)) {
                new_pyro_state.channels[0].is_armed = true;
                new_pyro_state.channels[0].is_firing = true;
                gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_ARM_PIN), HIGH);
                gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), HIGH);
            }
            break;
        case FSM_state::STATE_DROGUE_DEPLOY:
            // Fire "Pyro B" to deploy upper stage drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), HIGH);
            break;
        case FSM_state::STATE_MAIN_DEPLOY:
            // Fire "Pyro C" to deploy main.
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_FIRE_PIN), HIGH);
            break;
    }

    return new_pyro_state;
}

#endif

#ifdef IS_LOWER_STAGE

/**
 * Lower stage only!
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
PyroState Pyro::tick(FSMState fsm_state, Orientation orientation) {
    PyroState new_pyro_state = PyroState();

    // If the state is IDLE or any state after that, we arm the global arm pin
    if(fsm_state.curr_state >= FSM_state::STATE_IDLE) {
        new_pyro_state.is_global_armed = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), HIGH);
    }

    switch (fsm_state.curr_state) {
        case FSM_state::STATE_FIRST_STAGE_SEPERATION:
            // Fire "Pyro D" when seperating stage 1
            new_pyro_state.channels[3].is_armed = true;
            new_pyro_state.channels[3].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROD_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROD_FIRE_PIN), HIGH);
            break;
        case FSM_state::STATE_SUSTAINER_IGNITION:
            // Fire "Pyro A" to ignite sustainer
            new_pyro_state.channels[0].is_armed = true;
            new_pyro_state.channels[0].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), HIGH);
            break;
        case FSM_state::STATE_DROGUE_DEPLOY:
            // Fire "Pyro B" to deploy drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), HIGH);
            break;
        case FSM_state::STATE_MAIN_DEPLOY:
            // Fire "Pyro C" to deploy Main
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_ARM_PIN), HIGH);
            gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_FIRE_PIN), HIGH);
            break;

    }

    return new_pyro_state;
}

#endif

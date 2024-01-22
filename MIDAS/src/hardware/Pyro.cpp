#include "sensors.h"
#include "pins.h"

#include "TCAL9539.h"

#define MAXIMUM_TILT_ANGLE 20
#define GPIO_ID 0

/**
 * Helper function: Determines if the orientation is within an acceptable range to fire the second stage igniter.
*/
bool can_fire_igniter(Orientation orientation) {
    // This needs to be fleshed out. The sensor may not report angles in 'world space', so we need to determine
    // if the orientation of the rocket depends on other angles
    return std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE && std::abs(orientation.yaw) < MAXIMUM_TILT_ANGLE;
}

/**
 * "Initializes" the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 * have to do anything special.
*/
ErrorCode PyroThread::init() {
    bool gpio_init = TCAL9539Init();
    if(gpio_init) {
        // global arm
        gpioPinMode(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), OUTPUT);

        // arm pins
        gpioPinMode(GpioAddress(GPIO_ID, PYROA_ARM_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROB_ARM_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROC_ARM_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROD_ARM_PIN), OUTPUT);

        // fire pins
        gpioPinMode(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROC_ARM_PIN), OUTPUT);
        gpioPinMode(GpioAddress(GPIO_ID, PYROD_FIRE_PIN), OUTPUT);

        return ErrorCode::NoError;
    } else {
        return ErrorCode::PyroGPIOCouldNotBeInitialized;
    }
    
}

/**
 * Upper stage only!
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
Pyro PyroThread::tick_upper(FSMState fsm_state, Orientation orientation) {
    Pyro new_pyro = Pyro();

    // If the state is IDLE or any state after that, we arm the global arm pin
    if(fsm_state.curr_state >= FSM_state::STATE_IDLE) {
        new_pyro.is_global_armed = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), HIGH);
    }

    // Fire "Pyro A" to ignite sustainer
    // Additionally, check if orientation allows for firing
    if(fsm_state.curr_state == FSM_state::STATE_SUSTAINER_IGNITION && can_fire_igniter(orientation)) {
        new_pyro.channels[0].is_armed = true;
        new_pyro.channels[0].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), HIGH);
    }

    // Fire "Pyro B" to deploy upper stage drogue
    if(fsm_state.curr_state == FSM_state::STATE_DROGUE_DEPLOY) {
        new_pyro.channels[1].is_armed = true;
        new_pyro.channels[1].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), HIGH);
    }

    // Fire "Pyro C" to deploy upper stage main
    if(fsm_state.curr_state == FSM_state::STATE_MAIN_DEPLOY) {
        new_pyro.channels[2].is_armed = true;
        new_pyro.channels[2].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_FIRE_PIN), HIGH);
    }

    return new_pyro;
}

/**
 * Lower stage only!
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
 * Fires channels by setting their pin on the GPIO.
*/
Pyro PyroThread::tick_lower(FSMState fsm_state, Orientation orientation) {
    Pyro new_pyro = Pyro();

    // If the state is IDLE or any state after that, we arm the global arm pin
    if(fsm_state.curr_state >= FSM_state::STATE_IDLE) {
        new_pyro.is_global_armed = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYRO_GLOBAL_ARM_PIN), HIGH);
    }

    // Fire "Pyro D" when seperating stage 1
    if(fsm_state.curr_state == FSM_state::STATE_FIRST_STAGE_SEPERATION) {
        new_pyro.channels[3].is_armed = true;
        new_pyro.channels[3].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROD_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROD_FIRE_PIN), HIGH);
    }

    // Fire "Pyro A" to ignite sustainer
    if(fsm_state.curr_state == FSM_state::STATE_SUSTAINER_IGNITION) {
        new_pyro.channels[0].is_armed = true;
        new_pyro.channels[0].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROA_FIRE_PIN), HIGH);
    }

    // Fire "Pyro B" to deploy drogue
    if(fsm_state.curr_state == FSM_state::STATE_DROGUE_DEPLOY) {
        new_pyro.channels[1].is_armed = true;
        new_pyro.channels[1].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROB_FIRE_PIN), HIGH);
    }

    // Fire "Pyro C" to deploy Main
    if(fsm_state.curr_state == FSM_state::STATE_MAIN_DEPLOY) {
        new_pyro.channels[2].is_armed = true;
        new_pyro.channels[2].is_firing = true;
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_ARM_PIN), HIGH);
        gpioDigitalWrite(GpioAddress(GPIO_ID, PYROC_FIRE_PIN), HIGH);
    }

    return new_pyro;
}


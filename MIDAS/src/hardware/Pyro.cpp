#include "sensors.h"
#include "pins.h"

#include "TCAL9539-Q1.h"

#define MAXIMUM_TILT_ANGLE 20

GPIOTCAL9539 gpio_expander{};

/**
 * "Initializes" the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 * have to do anything special.
*/
ErrorCode PyroThread::init() {
    bool gpio_init = gpio_expander.init();
    if(gpio_init) {
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
        new_pyro.global_armed = true;
        gpio_expander.write(PYRO_GLOBAL_ARM_PIN, HIGH);
    }

    // Fire "Pyro A" to ignite sustainer
    // Additionally, check if orientation allows for firing
    if(fsm_state.curr_state == FSM_state::STATE_SUSTAINER_IGNITION && std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE) {
        new_pyro.channels[0].armed = true;
        new_pyro.channels[0].firing = true;
        gpio_expander.write(PYROA_ARM_PIN, HIGH);
        gpio_expander.write(PYROA_FIRE_PIN, HIGH);
    }

    // Fire "Pyro B" to deploy upper stage drogue
    if(fsm_state.curr_state == FSM_state::STATE_DROGUE_DEPLOY) {
        new_pyro.channels[1].armed = true;
        new_pyro.channels[1].firing = true;
        gpio_expander.write(PYROB_ARM_PIN, HIGH);
        gpio_expander.write(PYROB_FIRE_PIN, HIGH);
    }

    // Fire "Pyro C" to deploy upper stage main
    if(fsm_state.curr_state == FSM_state::STATE_MAIN_DEPLOY) {
        new_pyro.channels[2].armed = true;
        new_pyro.channels[2].firing = true;
        gpio_expander.write(PYROC_ARM_PIN, HIGH);
        gpio_expander.write(PYROC_FIRE_PIN, HIGH);
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
        new_pyro.global_armed = true;
        gpio_expander.write(PYRO_GLOBAL_ARM_PIN, HIGH);
    }

    // Fire "Pyro D" when seperating stage 1
    if(fsm_state.curr_state == FSM_state::STATE_FIRST_STAGE_SEPERATION) {
        new_pyro.channels[3].armed = true;
        new_pyro.channels[3].firing = true;
        gpio_expander.write(PYROD_ARM_PIN, HIGH);
        gpio_expander.write(PYROD_FIRE_PIN, HIGH);
    }

    // Fire "Pyro A" to ignite sustainer
    if(fsm_state.curr_state == FSM_state::STATE_SUSTAINER_IGNITION) {
        new_pyro.channels[0].armed = true;
        new_pyro.channels[0].firing = true;
        gpio_expander.write(PYROA_ARM_PIN, HIGH);
        gpio_expander.write(PYROA_FIRE_PIN, HIGH);
    }

    // Fire "Pyro B" to deploy drogue
    if(fsm_state.curr_state == FSM_state::STATE_DROGUE_DEPLOY) {
        new_pyro.channels[1].armed = true;
        new_pyro.channels[1].firing = true;
        gpio_expander.write(PYROB_ARM_PIN, HIGH);
        gpio_expander.write(PYROB_FIRE_PIN, HIGH);
    }

    // Fire "Pyro C" to deploy Main
    if(fsm_state.curr_state == FSM_state::STATE_MAIN_DEPLOY) {
        new_pyro.channels[2].armed = true;
        new_pyro.channels[2].firing = true;
        gpio_expander.write(PYROC_ARM_PIN, HIGH);
        gpio_expander.write(PYROC_FIRE_PIN, HIGH);
    }

    return new_pyro;
}


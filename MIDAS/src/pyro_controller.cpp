#include "pyro_controller.h"

#define MAXIMUM_TILT_ANGLE (M_PI/9)    // 20 degrees

PyroController::PyroController(IPyroBackend& backend) : backend(backend) { }

ErrorCode PyroController::init() {
    return backend.init();
}

/**
 * @brief Determines if orientation is in an acceptable range to ignite second stage.
 *
 * @return True if acceptable, false if not.
 */
bool can_fire_igniter(OrientationData orientation) {
    // This needs to be fleshed out. The sensor may not report angles in 'world space', so we need to determine
    // if the orientation of the rocket depends on other angles
    //    return std::abs(orientation.pitch) < MAXIMUM_TILT_ANGLE && std::abs(orientation.yaw) < MAXIMUM_TILT_ANGLE;
    (void) orientation;
    return true;
}

PyroState PyroController::tick_pyro(FSMState fsm_state, OrientationData orientation) {
    PyroState new_pyro_state = PyroState();

    if (fsm_state < FSMState::STATE_IDLE) {
        return new_pyro_state;
    }

    // If the state is IDLE or any state after that, we arm the global arm pin
    new_pyro_state.is_global_armed = true;
    backend.arm_all();

#ifdef IS_SUSTAINER
    switch (fsm_state) {
        case FSMState::STATE_SUSTAINER_IGNITION: {
            // Fire "Pyro A" to ignite sustainer
            // Additionally, check if orientation allows for firing
            if (can_fire_igniter(orientation)) {
                new_pyro_state.channels[0].is_armed = true;
                new_pyro_state.channels[0].is_firing = true;
                backend.fire_channel(0);
                // gpioDigitalWrite(PYROA_ARM_PIN, HIGH);
                // gpioDigitalWrite(PYROA_FIRE_PIN, HIGH);
            }
            break;
        }
        case FSMState::STATE_DROGUE_DEPLOY: {
            // Fire "Pyro B" to deploy upper stage drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            backend.fire_channel(1);
            // gpioDigitalWrite(PYROB_ARM_PIN, HIGH);
            // gpioDigitalWrite(PYROB_FIRE_PIN, HIGH);
            break;
        }
        case FSMState::STATE_MAIN_DEPLOY: {
            // Fire "Pyro C" to deploy main.
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            backend.fire_channel(2);
            // gpioDigitalWrite(PYROC_ARM_PIN, HIGH);
            // gpioDigitalWrite(PYROC_FIRE_PIN, HIGH);
            break;
        }
        default: break;
    }
#else
    switch (fsm_state) {
        case FSMState::STATE_FIRST_SEPARATION: {
            // Fire "Pyro D" when separating stage 1
            new_pyro_state.channels[3].is_armed = true;
            new_pyro_state.channels[3].is_firing = true;
            backend.fire_channel(3);
            break;
        }
        case FSMState::STATE_DROGUE_DEPLOY: {
            // Fire "Pyro B" to deploy drogue
            new_pyro_state.channels[1].is_armed = true;
            new_pyro_state.channels[1].is_firing = true;
            backend.fire_channel(1);
            break;
        }
        case FSMState::STATE_MAIN_DEPLOY: {
            // Fire "Pyro C" to deploy Main
            new_pyro_state.channels[2].is_armed = true;
            new_pyro_state.channels[2].is_firing = true;
            backend.fire_channel(2);
            break;
        }
        default: break;
    }
#endif

    return new_pyro_state;
}

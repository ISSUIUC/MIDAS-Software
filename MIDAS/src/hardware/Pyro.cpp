#include <cmath>

#include "sensors.h"
#include "pins.h"

#include "TCAL9538.h"
#include <flight-systems/rocket_state.h>
#include "finite-state-machines/pyro_eval.h"


// Fire the pyros for this time during PYRO_TEST (ms)
#define PYRO_TEST_FIRE_TIME 100

/**
 * @brief Returns true if the error_code signals failure.
 */
bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
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
    for(int i = 0; i < MIDAS_NUM_PYROS; ++i) {
        has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_PINS[i], OUTPUT));
        pyro_event_consumed[i] = false;
        pyro_event_check[i] = false;
        pyro_trigger_times[i] = 0;
    }


//    if (has_failed_gpio_init) {
//        return ErrorCode::PyroGPIOCouldNotBeInitialized;
//    } else {
    return ErrorCode::NoError;   // GPIO Driver always claimes it errored even when it doesn't.
//    }
}

void Pyro::disarm_all_channels(PyroState& prev_state) {
    for(int i = 0; i < MIDAS_NUM_PYROS; ++i) {
        prev_state.channel_firing[i] = false;
    }
    prev_state.is_global_armed = false;
}

void Pyro::set_pyro_safety() {
    safety_pyro_start_firing_time = pdTICKS_TO_MS(xTaskGetTickCount());
    safety_has_fired_pyros_this_cycle = true;
}

void Pyro::reset_pyro_safety() {
    safety_has_fired_pyros_this_cycle = false;
}

/**
 * @brief Fires channels by setting their desired fire state.
 * 
 * @return A pyro struct indicating which pyro channels are armed and/or firing.
 */
PyroState Pyro::tick(PyroTickData& data) {
    PyroState new_pyro_state = PyroState();
    double current_time = data.current_time;

    if (data.fsm.state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // If the state is not SAFE, we arm the global arm pin
    new_pyro_state.is_global_armed = true;

    if (data.fsm.state == FSMState::STATE_ARMED) { reset_pyro_safety(); return new_pyro_state; }
    if (data.fsm.state == FSMState::STATE_PYRO_TEST) {
        if(safety_has_fired_pyros_this_cycle) {
            // If a fire pyro command has already be acknowledged, do not acknowledge more commands, just fire pyro for the defined time
            // then, transition to SAFE.
            if((current_time - safety_pyro_start_firing_time) >= PYRO_TEST_FIRE_TIME) {
                data.commands.should_transition_safe = true;
                disarm_all_channels(new_pyro_state);
                data.commands.should_fire_pyro_a = false;
                data.commands.should_fire_pyro_b = false;
                data.commands.should_fire_pyro_c = false;
                data.commands.should_fire_pyro_d = false;

                reset_pyro_safety();
            }
            return new_pyro_state;
        }

        // Respond to telem commands to fire igniters
        if(data.commands.should_fire_pyro_a) {
            new_pyro_state.channel_firing[0] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_b) {
            // Fire pyro channel "B"
            new_pyro_state.channel_firing[1] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_c) {
            // Fire pyro channel "C"
            new_pyro_state.channel_firing[2] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_d) {
            // Fire pyro channel "D"
            new_pyro_state.channel_firing[3] = true;
            set_pyro_safety();
        }
        return new_pyro_state;
    }

    if(data.fsm_configuration.crc32 == FSM_CRC_FAIL_STATE) {
        return new_pyro_state;
    }

    // Load state
    PyroEvalState eval_state;
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        eval_state.trigger_times[i] = pyro_trigger_times[i];
        eval_state.event_check[i] = pyro_event_check[i];
        eval_state.event_consumed[i] = pyro_event_consumed[i];
    }

    // Calculate new state
    double tilt_deg = data.akf.mq_tilt * (180.0 / M_PI);
    PyroEvalResult eval = pyro_eval(
        data.fsm_configuration, data.fsm, tilt_deg,
        data.time_since_launch, data.ekf.velocity.vx,
        current_time, eval_state
    );

    // Write back state
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        pyro_trigger_times[i] = eval_state.trigger_times[i];
        pyro_event_check[i] = eval_state.event_check[i];
        pyro_event_consumed[i] = eval_state.event_consumed[i];
        new_pyro_state.channel_firing[i] = eval.channel_firing[i];
        new_pyro_state.pyro_event_consumed[i] = eval.event_consumed[i];
    }

    return new_pyro_state;
}

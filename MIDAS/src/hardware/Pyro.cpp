#include <cmath>

#include "sensors.h"
#include "pins.h"

#include "TCAL9538.h"
#include <flight-systems/rocket_state.h>
#include "finite-state-machines/pyro_eval.h"


// Fire the pyros for this time during PYRO_TEST (ms)
#define PYRO_TEST_FIRE_TIME 100

/**
 * @brief Returns whether a GPIO operation resulted in an error.
 *
 * @param error_code Result returned by a GPIO operation.
 *
 * @return True if the operation failed.
 */
bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
}

/**
 * @brief Initializes the pyro subsystem.
 *
 * Configures the global arm pin and each pyro channel as outputs and
 * resets all internal firing state. The GPIO driver currently reports
 * erroneous failures, so this function always returns NoError.
 *
 * @return ErrorCode::NoError.
 */
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // Configure the global arm output.
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // Configure each pyro channel and clear its runtime state.
    for(int i = 0; i < MIDAS_NUM_PYROS; ++i) {
        has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_PINS[i], OUTPUT));
        pyro_event_consumed[i] = false;
        pyro_event_check[i] = false;
        pyro_trigger_times[i] = 0;
    }

    // The GPIO driver always reports an error even when initialization
    // succeeds, so ignore the reported status for now.
    return ErrorCode::NoError;
}

/**
 * @brief Disarms every pyro channel.
 *
 * Clears all channel firing outputs and disables the global arm signal.
 *
 * @param prev_state Pyro state to modify.
 */
void Pyro::disarm_all_channels(PyroState& prev_state) {

    // Disable all individual pyro channels.
    for(int i = 0; i < MIDAS_NUM_PYROS; ++i) {
        prev_state.channel_firing[i] = false;
    }

    // Disable the global arm output.
    prev_state.is_global_armed = false;
}

/**
 * @brief Starts the pyro safety timer.
 *
 * Records the time at which a manual pyro firing began and prevents
 * additional firing commands from being accepted during the same cycle.
 */
void Pyro::set_pyro_safety() {
    safety_pyro_start_firing_time = pdTICKS_TO_MS(xTaskGetTickCount());
    safety_has_fired_pyros_this_cycle = true;
}

/**
 * @brief Clears the pyro safety latch.
 *
 * Allows future manual pyro firing commands to be accepted.
 */
void Pyro::reset_pyro_safety() {
    safety_has_fired_pyros_this_cycle = false;
}

/**
 * @brief Computes the desired pyro firing state.
 *
 * Handles manual pyro testing, SAFE state behavior, and autonomous
 * flight-event pyro deployment using the configured FSM logic.
 *
 * @param data Current flight state, sensor estimates, and command data.
 *
 * @return Desired pyro output state for this update.
 */
PyroState Pyro::tick(PyroTickData& data) {

    PyroState new_pyro_state = PyroState();
    double current_time = data.current_time;

    // Never arm or fire pyros while in the SAFE state.
    if (data.fsm.state == FSMState::STATE_SAFE) {
        disarm_all_channels(new_pyro_state);
        return new_pyro_state;
    }

    // Arm the global pyro enable whenever we leave SAFE.
    new_pyro_state.is_global_armed = true;

    // In the ARMED state, only enable the global arm output.
    if (data.fsm.state == FSMState::STATE_ARMED) {
        reset_pyro_safety();
        return new_pyro_state;
    }

    // Handle manual pyro testing.
    if (data.fsm.state == FSMState::STATE_PYRO_TEST) {

        // Ignore additional commands while a test firing is already active.
        if(safety_has_fired_pyros_this_cycle) {

            // End the test once the configured firing duration expires.
            if((current_time - safety_pyro_start_firing_time) >= PYRO_TEST_FIRE_TIME) {

                data.commands.should_transition_safe = true;
                disarm_all_channels(new_pyro_state);

                // Clear any remaining fire commands.
                data.commands.should_fire_pyro_a = false;
                data.commands.should_fire_pyro_b = false;
                data.commands.should_fire_pyro_c = false;
                data.commands.should_fire_pyro_d = false;

                reset_pyro_safety();
            }

            return new_pyro_state;
        }

        if(data.commands.should_fire_pyro_a) {
            new_pyro_state.channel_firing[0] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_b) {
            new_pyro_state.channel_firing[1] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_c) {
            new_pyro_state.channel_firing[2] = true;
            set_pyro_safety();
        }

        if(data.commands.should_fire_pyro_d) {
            new_pyro_state.channel_firing[3] = true;
            set_pyro_safety();
        }

        return new_pyro_state;
    }

    // Ignore autonomous pyro events if the loaded FSM configuration is invalid.
    if(data.fsm_configuration.crc32 == FSM_CRC_FAIL_STATE) {
        return new_pyro_state;
    }

    // Populate the evaluator with the previous pyro state.
    PyroEvalState eval_state;
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        eval_state.trigger_times[i] = pyro_trigger_times[i];
        eval_state.event_check[i] = pyro_event_check[i];
        eval_state.event_consumed[i] = pyro_event_consumed[i];
    }

    // Compute the updated pyro state using the shared evaluator.
    double tilt_deg = data.akf.mq_tilt * (180.0 / M_PI);

    PyroEvalResult eval = pyro_eval(
        data.fsm_configuration,
        data.fsm,
        tilt_deg,
        data.time_since_launch,
        data.ekf.velocity.vx,
        current_time,
        eval_state
    );

    // Save the evaluator state and output firing commands.
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        pyro_trigger_times[i] = eval_state.trigger_times[i];
        pyro_event_check[i] = eval_state.event_check[i];
        pyro_event_consumed[i] = eval_state.event_consumed[i];

        new_pyro_state.channel_firing[i] = eval.channel_firing[i];
        new_pyro_state.pyro_event_consumed[i] = eval.event_consumed[i];
    }

    return new_pyro_state;
}
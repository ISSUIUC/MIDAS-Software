#pragma once

/**
 * @author Michael Karpov (2026)
 * @brief Evaluates pyro firing state for the MIDAS V2 FSM.
 */

#include <cmath>
#include "fsm.h"
#include "flight-systems/sensor_data.h"

/**
 * @struct PyroEvalState
 *
 * @brief Stores persistent runtime state for each pyro channel between
 *        consecutive calls to `pyro_eval()`.
 *
 * This structure tracks when each pyro event was first triggered, whether
 * its post-delay condition check has been performed, and whether the event
 * has already completed and should no longer be evaluated.
 */
struct PyroEvalState {
    /**
     * @brief Timestamp when each pyro event was first triggered.
     *
     * A value of 0 indicates the event has not yet been triggered.
     */
    double trigger_times[MIDAS_NUM_PYROS] = {};

    /**
     * @brief Indicates whether the post-delay validation check has occurred.
     */
    bool event_check[MIDAS_NUM_PYROS] = {};

    /**
     * @brief Indicates whether each pyro event has finished and should no
     *        longer be evaluated.
     */
    bool event_consumed[MIDAS_NUM_PYROS] = {};
};

/**
 * @struct PyroEvalResult
 *
 * @brief Contains the evaluation results for each pyro channel after a
 *        single call to `pyro_eval()`.
 */
struct PyroEvalResult {
    /**
     * @brief True if the corresponding pyro event has been triggered.
     */
    bool event_triggered[MIDAS_NUM_PYROS] = {};

    /**
     * @brief True while the corresponding pyro output should be actively firing.
     */
    bool channel_firing[MIDAS_NUM_PYROS] = {};

    /**
     * @brief True if the corresponding pyro event has completed and should no
     *        longer be evaluated.
     */
    bool event_consumed[MIDAS_NUM_PYROS] = {};
};

/**
 * @brief Evaluates the state of every configured pyro channel.
 *
 * This function manages the complete lifecycle of each pyro event,
 * including trigger detection, delay timing, optional condition
 * revalidation, output firing, event completion, and soft reset handling.
 *
 * The supplied `PyroEvalState` is updated in-place and should be preserved
 * between consecutive calls throughout flight.
 *
 * @param config FSM configuration containing pyro actions and timing thresholds.
 * @param fsm Current finite state machine data.
 * @param tilt_deg Current rocket tilt angle in degrees.
 * @param time_since_launch Elapsed time since launch in seconds.
 * @param vx Current vertical velocity.
 * @param current_time Current system time used for timing delays.
 * @param state Persistent pyro evaluation state.
 *
 * @return A `PyroEvalResult` describing which events are triggered,
 *         actively firing, and consumed after this evaluation.
 */
inline PyroEvalResult pyro_eval(const FSMConfiguration& config, const FSMData& fsm, double tilt_deg, float time_since_launch, float vx, double current_time, PyroEvalState& state) {
    PyroEvalResult result = {};

    // Evaluate each configured pyro channel independently.
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        result.event_consumed[i] = state.event_consumed[i];

        // Skip pyros whose events have already completed.
        if (state.event_consumed[i]) {
            result.channel_firing[i] = false;
            result.event_triggered[i] = false;
            continue;
        }

        const FSMPyroAction& act = config.pyro_actions[i];

        // Wait for the pyro's trigger conditions to become true.
        if (state.trigger_times[i] == 0) {
            if (act.conditions_met(fsm.state, tilt_deg, fsm.current_motor, time_since_launch, vx)) {
                // Record the trigger time to begin the delay countdown.
                state.trigger_times[i] = current_time;
                result.event_triggered[i] = true;
            }
        } else {
            // The event has already been triggered.
            result.event_triggered[i] = true;

            // Wait until the configured firing delay has elapsed.
            if (current_time - state.trigger_times[i] >= act.delay) {

                // Before firing, optionally verify that the trigger
                // conditions are still satisfied. If they are no longer
                // valid, cancel the event permanently.
                if (!state.event_check[i] && !act.conditions_met(fsm.state, tilt_deg, fsm.current_motor, time_since_launch, vx)) {
                    state.event_consumed[i] = true;
                    result.event_consumed[i] = true;
                    result.channel_firing[i] = false;
                    continue;
                }

                // Mark that the verification step has completed and
                // energize the pyro channel.
                state.event_check[i] = true;
                result.channel_firing[i] = true;

                // Stop firing once the configured pulse duration expires.
                if (current_time - state.trigger_times[i] >= act.delay + config.thresholds.pyro_fire_t) {
                    state.event_consumed[i] = true;
                    result.event_consumed[i] = true;
                    result.channel_firing[i] = false;
                }
            } else {
                // Prior to the firing delay expiring, allow the event to
                // reset if the FSM leaves the qualifying state.
                if(!act.soft_conditions_met(fsm.state)) {
                    // Soft reset the pyro event.
                    state.trigger_times[i] = 0;
                }
            }
        }
    }

    return result;
}
#pragma once

/**
 * @author Michael Karpov (2026)
 * @brief Evaluates pyro firing state for the MIDAS V2 FSM
 */

#include <cmath>
#include "fsm.h"
#include "sensor_data.h"

struct PyroEvalState {
    double trigger_times[MIDAS_NUM_PYROS] = {};
    bool event_check[MIDAS_NUM_PYROS] = {};
    bool event_consumed[MIDAS_NUM_PYROS] = {};
};

struct PyroEvalResult {
    bool event_triggered[MIDAS_NUM_PYROS] = {};
    bool channel_firing[MIDAS_NUM_PYROS] = {};
    bool event_consumed[MIDAS_NUM_PYROS] = {};
};

/**
 * @brief Shared in-flight eval logic
 */
inline PyroEvalResult pyro_eval(const FSMConfiguration& config, const FSMData& fsm, double tilt_deg, float time_since_launch, float vx, double current_time, PyroEvalState& state) {
    PyroEvalResult result = {};

    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        result.event_consumed[i] = state.event_consumed[i];
        if (state.event_consumed[i]) {
            result.channel_firing[i] = false;
            result.event_triggered[i] = false;
            continue;
        }

        const FSMPyroAction& act = config.pyro_actions[i];

        if (state.trigger_times[i] == 0) {
            if (act.conditions_met(fsm.state, tilt_deg, fsm.current_motor, time_since_launch, vx)) {
                state.trigger_times[i] = current_time;
                result.event_triggered[i] = true;
            }
        } else {
            result.event_triggered[i] = true;
            if (current_time - state.trigger_times[i] >= act.delay) {
                if (!state.event_check[i] && !act.conditions_met(fsm.state, tilt_deg, fsm.current_motor, time_since_launch, vx)) {
                    state.event_consumed[i] = true;
                    result.event_consumed[i] = true;
                    result.channel_firing[i] = false;
                    continue;
                }

                state.event_check[i] = true;
                result.channel_firing[i] = true;

                if (current_time - state.trigger_times[i] >= act.delay + config.thresholds.pyro_fire_t) {
                    state.event_consumed[i] = true;
                    result.event_consumed[i] = true;
                    result.channel_firing[i] = false;
                }
            } else {
                if(!act.soft_conditions_met(fsm.state)) {
                    // Soft reset the pyro event.
                    state.trigger_times[i] = 0;
                }
            }
        }
    }

    return result;
}

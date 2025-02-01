#include "simulation.h"

#include "../emulation.h"

double SimulatedMotor::get_thrust(bool was_ignited, double time_since_ignition) const {
    if (!was_ignited) {
        return 0.0;
    }
    if (time_since_ignition < burn_time) {
        return max_thrust * (burn_time - time_since_ignition) / burn_time;
    } else {
        return 0;
    }
}

SimulatedMotor::SimulatedMotor(double max_thrust, double burn_time) : max_thrust(max_thrust), burn_time(burn_time) { }

void Simulation::step_rocket(RocketState& state, RocketParameters const& parameters, double dt) const {
    double thrust = parameters.motor.get_thrust(true, silsim_current_time() - parameters.ignition_time);
    double drag = 0.5 * density_of_air * parameters.drag_coefficient * parameters.cross_sectional_area * state.velocity * state.velocity;
    if (state.velocity < 0) {
        drag = -drag;
    }

    state.acceleration = (thrust - drag) / parameters.mass - gravity;
    state.velocity += state.acceleration * dt;
    state.height += state.velocity * dt;
    if (state.height < 0) {
        state.height = 0;
    }
}

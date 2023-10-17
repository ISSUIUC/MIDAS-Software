#include "simulation.h"

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


void Simulation::step(double dt) {
    for (SimulatedRocket& rocket : rockets) {
        rocket.step(current_time, dt);
    }
    current_time += dt;
}

void SimulatedRocket::step(double current_time, double dt) {
    if (!is_active) {
        return;
    }

    double thrust = parameters.motor.get_thrust(was_ignited, current_time - ignition_time);
    double drag = 0.5 * parameters.drag_coefficient * parameters.cross_sectional_area * velocity * velocity;

    velocity += (thrust - drag) / parameters.mass * dt;
    height += velocity * dt;
}

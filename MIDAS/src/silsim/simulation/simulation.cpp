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

SimulatedMotor::SimulatedMotor(double max_thrust, double burn_time) : max_thrust(max_thrust), burn_time(burn_time) { }


void Simulation::step(double dt) {
    for (SimulatedRocket* rocket : rockets) {
        rocket->step(current_time, dt);
    }
    current_time += dt;
}

void SimulatedRocket::step(double current_time, double dt) {
    if (!is_active) {
        return;
    }

    double thrust = parameters.motor.get_thrust(was_ignited, current_time - ignition_time);
    double drag = 0.5 * parameters.drag_coefficient * parameters.cross_sectional_area * velocity * velocity;

    acceleration = (thrust - drag) / parameters.mass;
    velocity += acceleration * dt;
    height += velocity * dt;
}

SimulatedRocket::SimulatedRocket(bool is_active, RocketParameters parameters) : is_active(is_active), parameters(parameters) { }

RocketParameters::RocketParameters(double mass, double area, double drag, SimulatedMotor motor) : mass(mass), cross_sectional_area(area), drag_coefficient(drag),
                                                                                                  motor(motor) { }

#include "simulation.h"

#include <utility>

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
        rocket->step(simulation_parameters, current_time, dt);
        if (rocket->height < 0) {
            rocket->height = 0;
        }
    }
    current_time += dt;
}

void Simulation::ignite_rocket(SimulatedRocket* rocket) {
    rocket->was_ignited = true;
    rocket->ignition_time = current_time;
}

void Simulation::activate_rocket(SimulatedRocket* rocket) {
    rocket->is_active = true;
}

void Simulation::deactivate_rocket(SimulatedRocket* rocket) {
    rocket->is_active = false;
}

Simulation::Simulation(SimulationParameters params, std::vector<SimulatedRocket*> rockets)
    : simulation_parameters(params), rockets(std::move(rockets)), current_time(0.0) {

}

void SimulatedRocket::step(SimulationParameters& sim_parameters, double current_time, double dt) {
    if (!is_active) {
        return;
    }

    double thrust = parameters.motor.get_thrust(was_ignited, current_time - ignition_time);
    double drag = 0.5 * sim_parameters.density_of_air * parameters.drag_coefficient * parameters.cross_sectional_area * velocity * velocity;
    if (velocity < 0) {
        drag = -drag;
    }

    acceleration = (thrust - drag) / parameters.mass - sim_parameters.gravity;
    velocity += acceleration * dt;
    height += velocity * dt;
}

SimulatedRocket::SimulatedRocket(bool is_active, RocketParameters parameters) : is_active(is_active), parameters(parameters) { }

void SimulatedRocket::copy_state_from(SimulatedRocket* other) {
    acceleration = other->velocity;
    velocity = other->velocity;
    height = other->height;
}

RocketParameters::RocketParameters(double mass, double area, double drag, SimulatedMotor motor) : mass(mass), cross_sectional_area(area), drag_coefficient(drag),
                                                                                                  motor(motor) { }

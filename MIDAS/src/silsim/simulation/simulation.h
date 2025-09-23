#pragma once

#include <vector>

struct SimulatedMotor {
    SimulatedMotor(double max_thrust, double burn_time);

    double get_thrust(bool was_ignited, double time_since_ignition) const;

private:
    double max_thrust;
    double burn_time;
};

struct RocketParameters {
    RocketParameters(double mass, double area, double drag, SimulatedMotor motor);

    double mass;
    double cross_sectional_area;
    double drag_coefficient;
    SimulatedMotor motor;
};

struct SimulationParameters {
    double density_of_air;
    double gravity_vec;
};

struct SimulatedRocket {
    SimulatedRocket(bool is_active, RocketParameters parameters);

    const RocketParameters parameters;

    bool is_active;
    double acceleration = 0.0;
    double velocity = 0.0;
    double height = 0.0;
    bool was_ignited = false;
    double ignition_time = 0.0;

    void step(SimulationParameters& sim_parameters, double current_time, double dt);
    void copy_state_from(SimulatedRocket* other);
};

struct Simulation {
    double current_time;
    SimulationParameters simulation_parameters;
    std::vector<SimulatedRocket*> rockets;

    Simulation(SimulationParameters params, std::vector<SimulatedRocket*> rockets);

    void step(double dt);

    void activate_rocket(SimulatedRocket* rocket);
    void deactivate_rocket(SimulatedRocket* rocket);
    void ignite_rocket(SimulatedRocket* rocket);
};
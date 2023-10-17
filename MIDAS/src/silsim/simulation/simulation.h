#pragma once

#include <vector>

struct SimulatedMotor {
    double get_thrust(bool was_ignited, double time_since_ignition) const;

private:
    double max_thrust;
    double burn_time;
};

struct RocketParameters {
    double mass;
    double cross_sectional_area;
    double drag_coefficient;
    SimulatedMotor motor;
};

struct SimulatedRocket {
    const RocketParameters parameters;

    bool is_active;
    double velocity;
    double height;
    bool was_ignited;
    double ignition_time;

    void step(double current_time, double dt);
};

struct Simulation {
    double current_time;
    std::vector<SimulatedRocket> rockets;

    void step(double dt);
};
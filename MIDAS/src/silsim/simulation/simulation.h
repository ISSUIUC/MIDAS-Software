#pragma once

struct SimulatedMotor {
    SimulatedMotor(double max_thrust, double burn_time);

    double get_thrust(bool was_ignited, double time_since_ignition) const;

private:
    double max_thrust;
    double burn_time;
};

struct RocketParameters {
    double mass;
    double cross_sectional_area;
    double drag_coefficient;
    double ignition_time;
    SimulatedMotor motor;
};

struct RocketState {
    double acceleration = 0.0;
    double velocity = 0.0;
    double height = 0.0;
};

struct Simulation {
    double density_of_air;
    double gravity;

    void step_rocket(RocketState& state, RocketParameters const& parameters, double dt) const;
};

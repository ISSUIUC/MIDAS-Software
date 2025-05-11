#include "pid.hpp"

PID::PID(std::vector<double> k): {
    K_list = k;
}
virtual int PID::Compute(double error, Motor motor, double dt) {
    integral += error;
    double derivative = (error - prev_error) / dt;
    double PID_calc = K_list[0] * error + K_list[1] * integral + K_list[2] * derivative;
    prev_error = error;
    return std::round(PID_calc / (motor.GetStepSize() * motor.GetGearRatio()));
}
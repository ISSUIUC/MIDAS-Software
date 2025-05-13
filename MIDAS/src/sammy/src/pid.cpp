#include "pid.h"

PID::PID(std::vector<double> k, SAMMotor motor) : motor(motor) {
    K_list = k;
}

int PID::compute(double error, double dt) {
    integral += error;
    double derivative = (error - prev_error) / dt;
    double PID_calc = K_list[0] * error + K_list[1] * integral + K_list[2] * derivative;
    prev_error = error;
    return std::round(PID_calc / (motor.get_step_size() * motor.get_gear_ratio()));
}

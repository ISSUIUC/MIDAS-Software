#include <Eigen/Eigen>

#include <cmath>
#include <vector>

#include "motor.h"

class PID {
public:
    PID(std::vector<double> k, SAMMotor motor);
    int compute(double error, double dt);
private:
    std::vector<double> K_list;
    double prev_error = 0;
    double integral = 0;
    SAMMotor motor;
};


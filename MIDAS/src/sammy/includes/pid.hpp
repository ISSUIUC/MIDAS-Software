#ifndef PID_HPP
#define PID_HPP
#include "sammy.hpp"
#include <vector>
#include "motor.hpp"
class PID: public Controller {
    public:
        PID(std::vector<double> k);
        virtual int Compute(double error, Motor motor, double dt);
    private:
        double prev_error = 0;
        double integral = 0;
        double K_list;
};

#endif
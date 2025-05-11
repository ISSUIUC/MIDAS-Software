#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cmath>

class Motor {
    public:
        Motor() = default;
        Motor(double step_size, double gear_ratio):
            step_size_(step_size * M_PI / 180), 
            gear_ratio_(gear_ratio) {}
        const double GetStepSize() { return step_size_; };
        const double GetGearRatio() { return gear_ratio_; };
        

    private:
        double step_size_ = 0.0;
        double gear_ratio_ = 0.0;
};

#endif
#ifndef SAMMY_HPP
#define SAMMY_HPP

#include <vector>

#include "motor.hpp"
#include "coords.hpp"
#include "pid.hpp"

class Controller {
    public:
        Controller() = default;
        virtual int compute();
};

class Sammy {
    public: 
        Sammy(std::vector<double> pos_turr, const std::vector<double>& pos_rocket, Motor& motor1, Motor& motor2, PID& controller1, PID& controller2);
        std::vector<double> Update(const std::vector<double>& pos_rocket, double time);
        
    private:
        std::vector<double> pos_turr_GPS_;
        std::vector<double> pos_turr_ECEF_;
        std::vector<double> pos_rocket_ECEF_;
        std::vector<double> pos_rocket_ENU_;
        std::vector<Motor> motors_;
        std::vector<PID> controllers_;
        std::vector<double> ang_init_;
        std::vector<double> ang_turr_;
        double prev_time = 0.0;
};

#endif
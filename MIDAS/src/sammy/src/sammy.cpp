#include "sammy.hpp"

#include <cmath>
#include <vector>

Sammy::Sammy(std::vector<double> pos_turr, std::vector<double> pos_rocket, Motor& motor1, Motor& motor2, PID& controller1, PID& controller2):
    pos_turr_GPS_(pos_turr), 
    pos_turr_ECEF_(GPS_to_ECEF(pos_turr)),
    pos_rocket_ECEF_(GPS_to_ECEF(pos_rocket)), 
    pos_rocket_ENU_(ECEF_to_ENU(pos_turr_GPS_, pos_turr_ECEF, pos_rocket_ECEF)), 
    motors_({motor1, motor2}), 
    controllers_({controller1, controller2}), 
    ang_init_({std::atan2(pos_rocket_ENU[2], std::sqrt(pos_rocket_ENU[0] * pos_rocket_ENU[0] + pos_rocket_ENU[1] * pos_rocket_ENU[1])), 
               std::atan2(pos_rocket_ENU[1], pos_rocket_ENU[0])})
    ang_turr_({0, 0}) {}


std::vector<double> Sammy::Update(const std::vector<double>& pos_rocket, double time) {
    pos_rocket_ECEF_ = GPS_to_ECEF(pos_rocket);
    pos_rocket_ENU_ = ECEF_to_ENU(pos_turr_GPS_, pos_turr_ECEF_, pos_rocket_ECEF_);
    double pitch = std::atan2(pos_rocket_ENU[2], std::sqrt(pos_rocket_ENU[0] * pos_rocket_ENU[0] + pos_rocket_ENU[1] * pos_rocket_ENU[1]));
    double yaw = std::atan2(pos_rocket_ENU[1], pos_rocket_ENU[0]);
    std::pair<double, double> ang_abs = {pitch, yaw};
    std::pair<double, double> ang_k = {norm_pi(ang_abs.first - ang_init_[0]), norm_pi(ang_abs.second - ang_init_[1])};
    std::pair<double, double> error = {ang_k.first - ang_turr_[0], ang_k.second - ang_turr_[1]};
}
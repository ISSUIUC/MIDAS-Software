#include <Eigen/Dense>
#include <cmath>
#include "motor.h"
#include "pid.h"
#include "coords.hpp"
#include "motor.h"

class Sammy {
public:
    // pos_turr_GPS: [lat (deg), lon (deg), alt]
    // pos_rocket_GPS: initial rocket GPS for any seeding
    Sammy(const Eigen::Vector3d& pos_turr_GPS,
          const Eigen::Vector3d& pos_rocket_GPS,
          SAMMotor& motor1,
          SAMMotor& motor2,
          PID& controller1,
          PID& controller2);

    // Returns new turret angles [azimuth, elevation]
    Eigen::Vector2d update(const Eigen::Vector3d& pos_rocket_GPS, double time);

private:
    Eigen::Vector3d pos_turr_GPS_;
    Eigen::Vector3d pos_turr_ECEF_;
    Eigen::Vector3d pos_rocket_ECEF_;
    Eigen::Vector3d pos_rocket_ENU_;

    std::vector<SAMMotor> motors_;        // two motors: [azimuth, elevation]
    std::vector<PID>   controllers_;   // two PID controllers

    // Angles and control state: [azimuth, elevation]
    Eigen::Vector2d ang_init_;
    Eigen::Vector2d ang_turr_;
    Eigen::Vector2d ang_abs_;
    Eigen::Vector2d ang_k_;
    Eigen::Vector2d error_;
    Eigen::Vector2d ang_turr_prev_;
};


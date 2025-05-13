#include "sammy.h"

Sammy::Sammy(
    const Eigen::Vector3d& pos_turr_GPS,
    const Eigen::Vector3d& pos_rocket_GPS,
    SAMMotor& motor1,
    SAMMotor& motor2,
    PID& controller1,
    PID& controller2
) :
    pos_turr_GPS_(pos_turr_GPS),
    pos_turr_ECEF_(GPS_to_ECEF(pos_turr_GPS_)),
    pos_rocket_ECEF_(GPS_to_ECEF(pos_rocket_GPS)),
    pos_rocket_ENU_(ECEF_to_ENU(pos_turr_GPS_, pos_turr_ECEF_, pos_rocket_ECEF_)),
    motors_{motor1, motor2},
    controllers_{controller1, controller2}
{
    // Initial pitch and yaw angles
    double pitch0 = std::atan2(
        pos_rocket_ENU_.z(),
        std::sqrt(pos_rocket_ENU_.x()*pos_rocket_ENU_.x() + pos_rocket_ENU_.y()*pos_rocket_ENU_.y())
    );
    double yaw0 = std::atan2(pos_rocket_ENU_.y(), pos_rocket_ENU_.x());
    ang_init_ << pitch0, yaw0;
    ang_turr_.setZero();
}

Eigen::Vector2d Sammy::update(const Eigen::Vector3d& pos_rocket_GPS, double time) {
    // Update rocket position in ECEF & ENU
    pos_rocket_ECEF_ = GPS_to_ECEF(pos_rocket_GPS);
    pos_rocket_ENU_  = ECEF_to_ENU(pos_turr_GPS_, pos_turr_ECEF_, pos_rocket_ECEF_);

    // Compute pitch and yaw from ENU
    double pitch = std::atan2(
        pos_rocket_ENU_.z(),
        std::sqrt(pos_rocket_ENU_.x()*pos_rocket_ENU_.x() + pos_rocket_ENU_.y()*pos_rocket_ENU_.y())
    );
    double yaw = std::atan2(pos_rocket_ENU_.y(), pos_rocket_ENU_.x());

    // Absolute angle vector
    ang_abs_ << pitch, yaw;

    // Relative to initial
    ang_k_.x() = norm_pi(ang_abs_.x() - ang_init_.x());
    ang_k_.y() = norm_pi(ang_abs_.y() - ang_init_.y());

    // Error vs current turret angles
    error_ = ang_k_ - ang_turr_;

    // PID control and motor updates
    for (int i = 0; i < 2; ++i) {
        double u = controllers_[i].compute(error_(i), time);
        motors_[i].setSpeed(u);
        ang_k_(i) = u;
    }

    // Integrate turret angles
    ang_turr_prev_ = ang_turr_;
    ang_turr_ += ang_k_ * time;

    // Return new turret angles: [pitch, yaw]
    return ang_turr_;
}

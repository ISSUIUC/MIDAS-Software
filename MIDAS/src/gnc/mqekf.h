#pragma once

#include "sensor_data.h" // for sim
#include "Buffer.h"      // for sim
#include "constants.h"
#include <Eigen/Eigen>
#include "systems.h"

class QuaternionMEKF
{
public:
    QuaternionMEKF(const Eigen::Matrix<float, 3, 1> &sigma_a,
                   const Eigen::Matrix<float, 3, 1> &sigma_g,
                   const Eigen::Matrix<float, 3, 1> &sigma_m);
    QuaternionMEKF();

    void initialize(RocketSystems *arg);
    void initialize_from_acc_mag(Acceleration const &, Magnetometer const &);
    void time_update(Velocity const &gyro, float Ts);
    void measurement_update(Acceleration const &acc, Magnetometer const &mag);
    void measurement_update_partial(
        Eigen::Matrix<float, 3, 1> const &meas,
        Eigen::Ref<Eigen::Matrix<float, 3, 1> const> const &vhat,
        Eigen::Ref<Eigen::Matrix<float, 3, 3> const> const &Rm);
        
    void tick(float dt, Magnetometer &magnetometer, Velocity &angular_velocity, Acceleration &acceleration, FSMState FSM_state);
    Eigen::Matrix<float, 4, 1> quaternion();
    Eigen::Matrix<float, 6, 6> covariance();
    Eigen::Matrix<float, 3, 1> gyroscope_bias();
    Eigen::Matrix<float, 3, 1> accelerometer_measurement_func() const;
    Eigen::Matrix<float, 3, 3> Racc, Rmag;
    AngularKalmanData getState();
    Eigen::Matrix<float, 3, 1> quatToEuler(const Eigen::Matrix<float, 4, 1> &q);
    // void tick(float dt, float sd, );
    void calculate_tilt();
    void set_transition_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>> &gyr, float Ts);
    Eigen::Matrix<float, 3, 3> skew_symmetric_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>> &vec) const;
    Eigen::Matrix<float, 3, 1> magnetometer_measurement_func() const;
    static Eigen::Matrix<float, 6, 6> initialize_Q(Eigen::Matrix<float, 3, 1> sigma_g);

private:
    Eigen::Quaternion<float> qref;
    AngularKalmanData state;
    float prev_tilt;

    Eigen::Matrix<float, 3, 1> sigma_a;
    Eigen::Matrix<float, 3, 1> sigma_g;
    Eigen::Matrix<float, 3, 1> sigma_m;
    Eigen::Matrix<float, 3, 1> v1ref;
    Eigen::Matrix<float, 3, 1> v2ref;

    // State
    Eigen::Matrix<float, 6, 1> x;
    // State covariance
    Eigen::Matrix<float, 6, 6> P;

    // Quaternion update matrix
    Eigen::Matrix<float, 4, 4> F;

    Eigen::Matrix<float, 6, 6> R;
    Eigen::Matrix<float, 6, 6> Q;
};

extern QuaternionMEKF mqekf;

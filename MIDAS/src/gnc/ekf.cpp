#include "ekf.h"
#include "finite-state-machines/fsm_states.h"
#include <Eigen/Dense>
#include <cmath>

const float GRAVITY = 9.80665f;

EKF::EKF() : KalmanFilter()
{
    state = KalmanData();
}

Eigen::Matrix3f EKF::eulerToRotation(float yaw, float pitch, float roll)
{
    yaw *= M_PI / 180.0f;
    pitch *= M_PI / 180.0f;
    roll *= M_PI / 180.0f;

    Eigen::Matrix3f Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw),  cos(yaw), 0,
          0,          0,        1;

    Eigen::Matrix3f Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0,          1, 0,
         -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    return Rz * Ry * Rx;
}

void EKF::initialize(RocketSystems *args)
{
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();

    // Initial altitude average
    float alt_sum = 0;
    for (int i = 0; i < 20; i++) {
        Barometer b = args->rocket_data.barometer.getRecent();
        alt_sum += b.altitude;
        // THREAD_SLEEP(50);
    }
    float alt0 = alt_sum / 20.0f;

    x_k.setZero();
    x_k(0) = alt0;  // x = up/down
    P_k.setIdentity();
    P_k *= 5.0f;    // initial covariance

    // Q: process noise
    float sigma_a = 1.0f;  // m/s²
    Q.setZero();
    for (int i = 0; i < 3; ++i) {
        Q(i, i) = 0.25 * pow(s_dt,4) * sigma_a*sigma_a;
        Q(i+3, i+3) = pow(s_dt,2) * sigma_a*sigma_a;
        Q(i+6, i+6) = sigma_a*sigma_a;
    }

    // R: measurement noise
    R.setZero();
    R(0,0) = 4.0f;      // barometer
    R(1,1) = 2.0f;      // accel X noise
    R(2,2) = 2.0f;      // accel Y noise
    R(3,3) = 2.0f;      // accel Z noise

    H.setZero();
    H(0,0) = 1.0f;      // position.x
}

void EKF::priori(float dt, Orientation &orientation, FSMState fsm)
{
    (void)orientation; 
    (void)fsm; 
    Eigen::Matrix<float,9,9> F_mat = Eigen::Matrix<float,9,9>::Identity();
    for (int i = 0; i < 3; ++i) {
        F_mat(i, i+3) = dt;
        F_mat(i, i+6) = 0.5f * dt * dt;
        F_mat(i+3, i+6) = dt;
    }

    x_priori = F_mat * x_k;
    P_priori = F_mat * P_k * F_mat.transpose() + Q;
}

void EKF::update(Barometer baro, Acceleration accel_body, Orientation orientation, FSMState fsm)
{
    (void)fsm;
    Eigen::Matrix<float,4,1> z;
    z(0) = baro.altitude;

    // rotation matrix
    euler_t e = orientation.getEuler();
    Eigen::Matrix3f R_be = eulerToRotation(e.yaw, e.pitch, e.roll);

    // accel in body frame
    //Eigen::Vector3f a_pred = R_be.transpose() * Eigen::Vector3f(0, 0, -GRAVITY) +  x_priori.segment<3>(6);
    Eigen::Vector3f a_pred = x_priori.segment<3>(6);

    z(1) = accel_body.ax;
    z(2) = accel_body.ay;
    z(3) = accel_body.az;

    // Predicted measurement
    Eigen::Matrix<float,4,1> z_pred;
    z_pred(0) = x_priori(0);    // altitude
    z_pred(1) = a_pred(0);
    z_pred(2) = a_pred(1);
    z_pred(3) = a_pred(2);

    // Jacobian =
    H.setZero();
    H(0,0) = 1.0f;  // baro altitude
    H(1,6) = 1.0f;  // accel x
    H(2,7) = 1.0f;  // accel y
    H(3,8) = 1.0f;  // accel z

    Eigen::Matrix<float,4,4> S = H * P_priori * H.transpose() + R;
    Eigen::Matrix<float,9,4> K = P_priori * H.transpose() * S.inverse();

    Eigen::Matrix<float,4,1> y = z - z_pred; 
    x_k = x_priori + K * y;
    P_k = (Eigen::Matrix<float,9,9>::Identity() - K * H) * P_priori;
}

// Was throwing errors so i added this
void EKF::priori()
{
    Orientation default_orientation;
    default_orientation.has_data = false;
    priori(s_dt, default_orientation, last_fsm);
}

KalmanData EKF::getState()
{
    KalmanData result;
    result.position.px = x_k(0);
    result.position.py = x_k(1);
    result.position.pz = x_k(2);
    result.velocity.vx = x_k(3);
    result.velocity.vy = x_k(4);
    result.velocity.vz = x_k(5);
    result.acceleration.ax = x_k(6);
    result.acceleration.ay = x_k(7);
    result.acceleration.az = x_k(8);
    result.altitude = x_k(0);
    return result;
}

void EKF::setState(KalmanState state)
{
    x_k(0) = state.state_est_pos_x;
    x_k(1) = state.state_est_pos_y;
    x_k(2) = state.state_est_pos_z;
    x_k(3) = state.state_est_vel_x;
    x_k(4) = state.state_est_vel_y;
    x_k(5) = state.state_est_vel_z;
    x_k(6) = state.state_est_accel_x;
    x_k(7) = state.state_est_accel_y;
    x_k(8) = state.state_est_accel_z;
}

void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state)
{
    priori(dt, orientation, state);
    update(barometer, acceleration, orientation, state);
    last_fsm = state;
}

// Placeholder implementations for other methods
void EKF::setQ(float dt, float sd)
{
    (void)dt; (void)sd; 
}

void EKF::setF(float dt, FSMState fsm, float w_x, float w_y, float w_z)
{
    (void)dt; (void)fsm; (void)w_x; (void)w_y; (void)w_z; 
}

void EKF::getThrust(float timestamp, euler_t angles, FSMState FSM_state, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)timestamp; (void)angles; (void)FSM_state; (void)to_modify; 
}

void EKF::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &x_k, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)angles; (void)x_k; (void)to_modify; 
}

void EKF::GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)angles; (void)to_modify;
}

float EKF::linearInterpolation(float x0, float y0, float x1, float y1, float x)
{
    if (x1 == x0) return y0;
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

EKF ekf;
#include "example_kf.h"

ExampleKalmanFilter::ExampleKalmanFilter() : KalmanFilter() {}

void ExampleKalmanFilter::initialize(RocketSystems* args) {}


void ExampleKalmanFilter::priori() {
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void ExampleKalmanFilter::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState current_stat) {}

void ExampleKalmanFilter::setQ(float dt, float sd) {}

void ExampleKalmanFilter::setF(float dt) {}

KalmanData ExampleKalmanFilter::getState() { return KalmanData(); }

void ExampleKalmanFilter::setState(KalmanState state)
{
    this->state.position.px = state.state_est_pos_x;
    this->state.position.py = state.state_est_pos_y;
    this->state.position.pz = state.state_est_pos_z;
    this->state.acceleration.ax = state.state_est_accel_x;
    this->state.acceleration.ay = state.state_est_accel_y;
    this->state.acceleration.az = state.state_est_accel_z;
    this->state.velocity.vx =state.state_est_vel_x;
    this->state.velocity.vy =state.state_est_vel_y;
    this->state.velocity.vz =state.state_est_vel_z;
}

ExampleKalmanFilter example_kf;

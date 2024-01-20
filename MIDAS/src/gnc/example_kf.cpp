#include "example_kf.h"

ExampleKalmanFilter::ExampleKalmanFilter() : KalmanFilter() {}

void ExampleKalmanFilter::initialize() {}


void ExampleKalmanFilter::priori() {
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void ExampleKalmanFilter::update() {}

void ExampleKalmanFilter::setQ(float dt, float sd) {}

void ExampleKalmanFilter::setF(float dt) {}

KalmanData ExampleKalmanFilter::getState() { return KalmanData(); }

void ExampleKalmanFilter::setState(KalmanData state)
{
    this->state = state;
}

ExampleKalmanFilter example_kf;

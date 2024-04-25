#pragma once

#include "kalman_filter.h"

// makes a kalman filter with 9 state variables and 3 sensor inputs
class ExampleKalmanFilter : public KalmanFilter<9, 3>
{
public:
    ExampleKalmanFilter();

    void initialize() override;
    void priori() override;
    void update() override;

    void setQ(float dt, float sd) override;
    void setF(float dt) override;

    KalmanData getState() override;
    void setState(KalmanData state) override;

    Eigen::Matrix<float, 3, 1> bodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

private:
    KalmanData state;
};

extern ExampleKalmanFilter example_kf;
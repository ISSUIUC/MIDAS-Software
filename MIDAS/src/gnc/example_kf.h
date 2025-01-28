#pragma once

#include "kalman_filter.h"

// makes a kalman filter with 9 state variables and 3 sensor inputs
class ExampleKalmanFilter : public KalmanFilter<9, 4>
{
public:
    ExampleKalmanFilter();

    void initialize(RocketSystems* args) override;
    //virtual void initialize(Orientation &orientation, Barometer &barometer, Acceleration &Acceleration);
    void priori() override;
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState current_state) override;

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void setQ(float dt, float sd);
    void setF(float dt);

    Eigen::Matrix<float, 3, 1> bodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

private:
    KalmanData state;
    KalmanState kalman_state;
};

extern ExampleKalmanFilter example_kf;

#pragma once
#include "../finite-state-machines/fsm_states.h"
#include "kalman_filter.h"
// #include "mcu_main/finite-state-machines/RocketFSMBase.h"

// makes a kalman filter with 3 state variables and 3 sensor inputs
class ExampleKalmanFilter : public KalmanFilter<3, 3>
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
    void kfTickFunction(FSM_state& curr_state, float dt);


private:
    float s_dt = 0.016; // 16 ms THREAD_SLEEP
    float spectral_density = 13.0;
    KalmanData state;

};

extern ExampleKalmanFilter example_kf;

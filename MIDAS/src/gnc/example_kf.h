#pragma once
#include "../finite-state-machines/fsm_states.h"
#include "kalman_filter.h"


#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 3

// #include "mcu_main/finite-state-machines/RocketFSMBase.h"

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
    void kfTickFunction(FSM_state& curr_state, Barometer& curr_baro_buf, HighGData& curr_accel, Orientation& curr_orientation, float dt);


    Eigen::Matrix<float, 3, 1> bodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

private:
    void updatePrivVars(FSM_state& curr_state, Barometer& curr_baro_buf, HighGData& curr_accel, Orientation& curr_orientation, float& dt);
    float s_dt = 0.016; // 16 ms THREAD_SLEEP
    float spectral_density = 13.0;
    KalmanData state;
    FSM_state _curr_state;
    Barometer _curr_baro_buf;
    HighGData _curr_accel;
    Orientation _curr_orientation;
    float _dt;
};

extern ExampleKalmanFilter example_kf;

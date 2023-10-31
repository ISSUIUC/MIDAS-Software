#pragma once
#include "../finite-state-machines/fsm_states.h"
#include "kalman_filter.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4

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

    Eigen::Matrix<float, NUM_STATES, 1> x_k = Eigen::Matrix<float, NUM_STATES, 1>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> F_mat = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_STATES> H = Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> P_k = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> Q = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_SENSOR_INPUTS> R = Eigen::Matrix<float, NUM_SENSOR_INPUTS, NUM_SENSOR_INPUTS>::Zero();  // Diagonal
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> P_priori = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Zero();
    Eigen::Matrix<float, NUM_STATES, 1> x_priori = Eigen::Matrix<float, NUM_STATES, 1>::Zero();
    Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS> K = Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS>::Zero();
    Eigen::Matrix<float, NUM_SENSOR_INPUTS, 1> y_k = Eigen::Matrix<float, NUM_SENSOR_INPUTS, 1>::Zero();

    Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS> B = Eigen::Matrix<float, NUM_STATES, NUM_SENSOR_INPUTS>::Zero();

};

extern ExampleKalmanFilter example_kf;

#pragma once

#undef abs
#undef F
#undef round
#undef B1

#include <Eigen/Eigen>
#include "sensor_data.h"


#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

typedef struct KalmanState {
    float state_est_pos_x;
    float state_est_vel_x;
    float state_est_accel_x;
    float state_est_pos_y;
    float state_est_vel_y;
    float state_est_accel_y;
    float state_est_pos_z;
    float state_est_vel_z;
    float state_est_accel_z;

    float state_est_r_pos_x;
    float state_est_r_vel_x;
    float state_est_r_accel_x;
    float state_est_r_pos_y;
    float state_est_r_vel_y;
    float state_est_r_accel_y;
    float state_est_r_pos_z;
    float state_est_r_vel_z;
    float state_est_r_accel_z;
} KalmanState;

template <int _NumStates, int _NumInputs>
class KalmanFilter
{
protected:
    Eigen::Matrix<float, _NumStates, 1> x_k;
    Eigen::Matrix<float, _NumStates, _NumStates> F_mat;
    Eigen::Matrix<float, _NumInputs, _NumStates> H;
    Eigen::Matrix<float, _NumStates, _NumStates> P_k;
    Eigen::Matrix<float, _NumStates, _NumStates> Q;
    Eigen::Matrix<float, _NumInputs, _NumInputs> R;
    Eigen::Matrix<float, _NumStates, _NumStates> P_priori;
    Eigen::Matrix<float, _NumStates, 1> x_priori;
    Eigen::Matrix<float, _NumStates, _NumInputs> K;
    Eigen::Matrix<float, _NumInputs, 1> y_k;

    Eigen::Matrix<float, _NumStates, _NumInputs> B;

public:
    KalmanFilter()
    {
        x_k = Eigen::Matrix<float, _NumStates, 1>::Zero();
        F_mat = Eigen::Matrix<float, _NumStates, _NumStates>::Zero();
        H = Eigen::Matrix<float, _NumInputs, _NumStates>::Zero();
        P_k = Eigen::Matrix<float, _NumStates, _NumStates>::Zero();
        Q = Eigen::Matrix<float, _NumStates, _NumStates>::Zero();
        R = Eigen::Matrix<float, _NumInputs, _NumInputs>::Zero(); // Diagonal
        P_priori = Eigen::Matrix<float, _NumStates, _NumStates>::Zero();
        x_priori = Eigen::Matrix<float, _NumStates, 1>::Zero();
        K = Eigen::Matrix<float, _NumStates, _NumInputs>::Zero();
        y_k = Eigen::Matrix<float, _NumInputs, 1>::Zero();

        B = Eigen::Matrix<float, _NumStates, _NumInputs>::Zero();
    }

    virtual void void initialize(Orientation &orientation, Barometer &barometer, Acceleration &Acceleration);
    virtual void priori();
    virtual void update(Barometer barometer, Acceleration acceleration, Orientation orientation);

    virtual KalmanData getState();
    virtual void setState(KalmanState state);
};

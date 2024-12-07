#pragma once

#undef abs
#undef F
#undef round
#undef B1

#include <Eigen/Eigen>
#include "sensor_data.h"
#include "systems.h" //ignore


struct KalmanState {    // class that keeps track of pos, vel, accel in xyz coordinates
    float state_est_pos_x;
    float state_est_vel_x;
    float state_est_accel_x;
    float state_est_pos_y;
    float state_est_vel_y;
    float state_est_accel_y;
    float state_est_pos_z;
    float state_est_vel_z;
    float state_est_accel_z;
};

template <int _NumStates, int _NumInputs>
// i understand states representing x,y,z pos,vel,accel
// numInputs are, well, inputs, but what inputs they usually are is unclear, maybe made clear in later parts
class KalmanFilter
{
protected:
    Eigen::Matrix<float, _NumStates, 1> x_k; // State Vector: current estimated state of system
    Eigen::Matrix<float, _NumStates, _NumStates> F_mat; // State Transition Matrix: shows how system evolves from one state to another
    Eigen::Matrix<float, _NumInputs, _NumStates> H; // Measurement matrix: relates state variables to observed measurements
    Eigen::Matrix<float, _NumStates, _NumStates> P_k; // Error Covariance Matrix : represents uncertainty in measurements at current step
    Eigen::Matrix<float, _NumStates, _NumStates> Q; // Process Noise Covariance : represents uncertainty introduced by model 
    Eigen::Matrix<float, _NumInputs, _NumInputs> R; // Measurement Noise Covariance : represents uncertainty in measurements (sensor noise)
    Eigen::Matrix<float, _NumStates, _NumStates> P_priori; // Priori Error Covariance : predicted uncertainty of state before measurement
    Eigen::Matrix<float, _NumStates, 1> x_priori; // Priori State Estimate : predicted state of system before measurements considered
    Eigen::Matrix<float, _NumStates, _NumInputs> K; // Kalman Gain : figures weight given to new measurements vs predictions during update phase
    Eigen::Matrix<float, _NumInputs, 1> y_k; // Measurement Residual : difference between predicted and actual measurements

    Eigen::Matrix<float, _NumStates, _NumInputs> B; // Control Matrix

public:
    KalmanFilter()
    {   // initializes everything to zero (who would have guessed....)
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
    // same here
    virtual void initialize(RocketSystems* args) = 0;
    virtual void priori() = 0;
    virtual void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState current_state) = 0;

    virtual KalmanData getState() = 0;
    virtual void setState(KalmanState state) = 0;
};

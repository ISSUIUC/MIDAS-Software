#pragma once

#undef abs
#undef F
#undef round
#undef B1

#include <Eigen/Eigen>
#include "sensor_data.h"
#include "systems.h"


struct KalmanState {
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
    Eigen::Matrix<float, _NumStates, 1> Wind ;
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
        Wind = Eigen::Matrix<float, _NumStates, 1>::Zero(); // Wind vector
        B = Eigen::Matrix<float, _NumStates, _NumInputs>::Zero();
        
    }
    
    virtual void initialize(RocketSystems* args) = 0;
    virtual void priori() = 0;
    virtual void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState current_state) = 0;
    virtual KalmanData getState() = 0;
    virtual void setState(KalmanState state) = 0;

    #ifdef GNC_DATA
    virtual void encode_to_buf(float* buf) {};
    #endif
};


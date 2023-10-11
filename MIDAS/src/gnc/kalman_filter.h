#pragma once

#undef abs
#undef F
#undef round
#undef B1

#include <Eigen/Eigen>
#include "sensors.h"

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

    virtual void initialize() = 0;
    virtual void priori() = 0;
    virtual void update() = 0;

    virtual void setQ(float dt, float sd) = 0;
    virtual void setF(float dt) = 0;

    virtual KalmanData getState() = 0;
    virtual void setState(KalmanData state) = 0;
};

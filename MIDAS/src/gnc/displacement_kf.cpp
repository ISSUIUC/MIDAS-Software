#include "displacement_kf.h"

#include <Eigen/Eigen>

DisplacementKalmanFilter displacement_kf;

DisplacementKalmanFilter::DisplacementKalmanFilter() : KalmanFilter()
{
    initialize();
}

void DisplacementKalmanFilter::initialize()
{
    // set initial transition matrix
    setF(s_dt_);
    // set initial process noise matrix
    setQ(s_dt_, spectral_density_);

    // barometer input for x displacement
    H(0,0) = 1;
    // accelerometer input for all accelerations
    H(1, 2) = 1;
    H(2, 5) = 1;
    H(3, 8) = 1;

    // set measurement uncertainties
    R(0, 0) = 2.0;
    R(1, 1) = 1.9;
    R(2, 2) = 10;
    R(3, 3) = 10;
}

void DisplacementKalmanFilter::initialize(Barometer& barometerData, Acceleration& imuData)
{
    // set initial state
    x_k(0,0) = barometerData.altitude;
    x_k(2,0) = imuData.ax;
    x_k(5,0) = imuData.ay;
    x_k(8,0) = imuData.az;

    initialize();
}

void DisplacementKalmanFilter::priori()
{
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void DisplacementKalmanFilter::update()
{
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;
}

void DisplacementKalmanFilter::setQ(float dt, float sd)
{
    for (int i = 0; i < 3; i++)
    {
        Q(3 * i, 3 * i) = pow(dt, 5) / 20;
        Q(3 * i, 3 * i + 1) = pow(dt, 4) / 8;
        Q(3 * i, 3 * i + 2) = pow(dt, 3) / 6;
        Q(3 * i + 1, 3 * i + 1) = pow(dt, 3) / 8;
        Q(3 * i + 1, 3 * i + 2) = pow(dt, 2) / 2;
        Q(3 * i + 2, 3 * i + 2) = dt;
        Q(1, 0) = this->Q(0, 1);
        Q(2, 0) = this->Q(0, 2);
        Q(2, 1) = this->Q(1, 2);
    }

    Q *= sd;
}

void DisplacementKalmanFilter::setF(float dt)
{
    for (int i = 0; i < 3; i++)
    {
        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i, 3 * i + 1) = dt;
        F_mat(3 * i, 3 * i + 2) = pow(dt, 2) / 2;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 1, 3 * i + 2) = dt;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

KalmanData DisplacementKalmanFilter::getState()
{
    KalmanData state;
    
    state.position.px = x_k(0, 0);
    state.position.py = x_k(3, 0);
    state.position.pz = x_k(6, 0);

    state.velocity.vx = x_k(1, 0);
    state.velocity.vy = x_k(4, 0);
    state.velocity.vz = x_k(7, 0);

    state.acceleration.ax = x_k(2, 0);
    state.acceleration.ay = x_k(5, 0);
    state.acceleration.az = x_k(8, 0);

    return state;
}

void DisplacementKalmanFilter::setState(KalmanData state)
{
    x_k(0, 0) = state.position.px;
    x_k(3, 0) = state.position.py;
    x_k(6, 0) = state.position.pz;

    x_k(1, 0) = state.velocity.vx;
    x_k(4, 0) = state.velocity.vy;
    x_k(7, 0) = state.velocity.vz;

    x_k(2, 0) = state.acceleration.ax;
    x_k(5, 0) = state.acceleration.ay;
    x_k(8, 0) = state.acceleration.az;
}

void DisplacementKalmanFilter::kfTick(float dt, float sd, Barometer& barometerData, Acceleration& imuData)
{
    setF(dt);
    setQ(dt, sd);
    priori();

    y_k(0, 0) = barometerData.altitude;
    y_k(1, 0) = imuData.ax;
    y_k(2, 0) = imuData.ay;
    y_k(3, 0) = imuData.az;

    update();

    // data needs to get normalized after filtering
    // don't subtract normal acceleration from gravity before filtering
}

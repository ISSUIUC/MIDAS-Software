#include "displacement_kf.h"

#include <Eigen/Eigen>

DisplacementKalmanFilter displacement_kf;

DisplacementKalmanFilter::DisplacementKalmanFilter() : KalmanFilter()
{
    this->initialize();
}

void DisplacementKalmanFilter::initialize()
{
    // set initial transition matrix
    this->setF(this->s_dt_);
    // set initial process noise matrix
    this->setQ(this->s_dt_, this->spectral_density_);

    // barometer input for x displacement
    this->H(0,0) = 1;
    // accelerometer input for all accelerations
    this->H(1, 2) = 1;
    this->H(2, 5) = 1;
    this->H(3, 8) = 1;

    // set measurement uncertainties
    this->R(0, 0) = 2.0;
    this->R(1, 1) = 1.9;
    this->R(2, 2) = 10;
    this->R(3, 3) = 10;
}

void DisplacementKalmanFilter::initialize(Barometer& barometerData, Acceleration& imuData)
{
    // set initial state
    this->x_k(0,0) = barometerData.altitude;
    this->x_k(2,0) = imuData.ax;
    this->x_k(5,0) = imuData.ay;
    this->x_k(8,0) = imuData.az;

    this->initialize();
}

void DisplacementKalmanFilter::priori()
{
    this->x_priori = (this->F_mat * this->x_k);
    this->P_priori = (this->F_mat * this->P_k * this->F_mat.transpose()) + this->Q;
}

void DisplacementKalmanFilter::update()
{
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    
    this->x_k = this->x_priori + this->K * (this->y_k - (this->H * this->x_priori));
    P_k = (identity - this->K * this->H) * this->P_priori;
}

void DisplacementKalmanFilter::setQ(float dt, float sd)
{
    for (int i = 0; i < 3; i++)
    {
        this->Q(3 * i, 3 * i) = pow(dt, 5) / 20;
        this->Q(3 * i, 3 * i + 1) = pow(dt, 4) / 8;
        this->Q(3 * i, 3 * i + 2) = pow(dt, 3) / 6;
        this->Q(3 * i + 1, 3 * i + 1) = pow(dt, 3) / 8;
        this->Q(3 * i + 1, 3 * i + 2) = pow(dt, 2) / 2;
        this->Q(3 * i + 2, 3 * i + 2) = dt;
        this->Q(1, 0) = this->Q(0, 1);
        this->Q(2, 0) = this->Q(0, 2);
        this->Q(2, 1) = this->Q(1, 2);
    }

    Q *= sd;
}

void DisplacementKalmanFilter::setF(float dt)
{
    for (int i = 0; i < 3; i++)
    {
        this->F_mat(3 * i, 3 * i) = 1;
        this->F_mat(3 * i, 3 * i + 1) = dt;
        this->F_mat(3 * i, 3 * i + 2) = pow(dt, 2) / 2;
        this->F_mat(3 * i + 1, 3 * i + 1) = 1;
        this->F_mat(3 * i + 1, 3 * i + 2) = dt;
        this->F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

KalmanData DisplacementKalmanFilter::getState()
{
    KalmanData state;
    
    state.position.px = this->x_k(0, 0);
    state.position.py = this->x_k(3, 0);
    state.position.pz = this->x_k(6, 0);

    state.velocity.vx = this->x_k(1, 0);
    state.velocity.vy = this->x_k(4, 0);
    state.velocity.vz = this->x_k(7, 0);

    state.acceleration.ax = this->x_k(2, 0);
    state.acceleration.ay = this->x_k(5, 0);
    state.acceleration.az = this->x_k(8, 0);

    return state;
}

void DisplacementKalmanFilter::setState(KalmanData state)
{
    this->x_k(0, 0) = state.position.px;
    this->x_k(3, 0) = state.position.py;
    this->x_k(6, 0) = state.position.pz;

    this->x_k(1, 0) = state.velocity.vx;
    this->x_k(4, 0) = state.velocity.vy;
    this->x_k(7, 0) = state.velocity.vz;

    this->x_k(2, 0) = state.acceleration.ax;
    this->x_k(5, 0) = state.acceleration.ay;
    this->x_k(8, 0) = state.acceleration.az;
}

void DisplacementKalmanFilter::kfTick(float dt, float sd, Barometer& barometerData, Acceleration& imuData)
{
    this->setF(dt);
    this->setQ(dt, sd);
    this->priori();

    this->y_k(0, 0) = barometerData.altitude;
    this->y_k(1, 0) = imuData.ax;
    this->y_k(2, 0) = imuData.ay;
    this->y_k(3, 0) = imuData.az;

    this->update();

    // data needs to get normalized after filtering
    // don't subtract normal acceleration from gravity before filtering
}

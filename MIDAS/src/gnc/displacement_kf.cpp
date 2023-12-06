#include "displacement_kf.h"

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
    this->x_k(0,0) = barometerData.pressure;
    this->x_k(2,0) = imuData.ax;
    this->x_k(5,0) = imuData.ay;
    this->x_k(8,0) = imuData.az;

    this->initialize();
}




#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "FifoBuffer.h"
//#include "systems.h"

class Yessir : public KalmanFilter<3, 3>
{
public:
    Yessir();
    void initialize(RocketSystems* args) override; // y is this getting args
    //void initialize(Orientation &orientation, Barometer &barometer, Acceleration &Acceleration) override; //Works
    void priori() override; //works
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;

    void setQ(float dt, float sd); //Works
    void setF(float dt); //Works
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
    void updateApogee(float estimate);

private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    KalmanState kalman_state;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 1> world_accel;
    FifoBuffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};

extern Yessir yessir;

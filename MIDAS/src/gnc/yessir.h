#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "FifoBuffer.h"

class Yessir : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    Yessir();
    void initialize(RocketSystems* args) override; 
    //void initialize(Orientation &orientation, Barometer &barometer, Acceleration &Acceleration) override; //Works
    void priori() override; 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;

    void setQ(float dt, float sd);
    void setF(float dt); 
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
   

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

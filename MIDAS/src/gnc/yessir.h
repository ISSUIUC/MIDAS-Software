#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

class Yessir final: public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS> {
public:
    Yessir();
    void initialize(RocketData& args) override;
    void priori() override; 
    void update(BarometerData barometer, Acceleration acceleration, OrientationData orientation, FSMState state) override;

    void setQ(float dt, float sd);
    void setF(float dt); 
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void tick(float dt, float sd, BarometerData &barometer, Acceleration acceleration, OrientationData &orientation, FSMState state);
   
    bool should_reinit = true;
private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    KalmanState kalman_state;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 1> world_accel;
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};

extern Yessir yessir;

#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h"

#define NUM_STATES 9 //x,y,z pos, vel, accel
#define NUM_SENSOR_INPUTS 4 // not sure what inputs are from sensors, maybe its baro, accel, orientation, and fsmState?
#define ALTITUDE_BUFFER_SIZE 10 // amount of collected sensor data it will hold on to at a time, this is make sure its collected data is "current"

class Yessir : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    Yessir();
    void initialize(RocketSystems* args) override; // prepares rocketsystem info to be used in calc
    void priori() override; // implements prediction step of filter
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;
    // updates using the given info above to refine state estimate

    void setQ(float dt, float sd);
    void setF(float dt); 
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
   
    bool should_reinit = false;
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

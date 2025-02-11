#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    void priori();
    void priori(float dt, Orientation &orientation, FSMState fsm); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;

    void setQ(float dt, float sd);
    void setF(float dt, FSMState fsm, float w_x, float w_y, float w_z); 
    Eigen::Matrix<float, 3, 1> *getThrust(float timestamp, euler_t angles, FSMState FSM_state);
    Eigen::Matrix<float, 3, 1> *BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> *x_k);
    Eigen::Matrix<float, 3, 1> *GlobalToBody(euler_t angles);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    float linearInterpolation(float x0, float y0, float x1, float y1, float x);

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
   
    bool should_reinit = false;
private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    float Ca = 0;
    float Cn = 0;
    float Cp = 0;
    Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};

extern EKF ekf;

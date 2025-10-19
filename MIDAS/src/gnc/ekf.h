#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h"
#include "constants.h"
#include "aero_coeff.h"
#include "rotation.h"

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

// Number of entries for aerodynamic data table
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    void priori();
    void priori(float dt, Orientation &orientation, FSMState fsm); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;

    void setQ(float dt, float sd);
    void setF(float dt, float w_x, float w_y, float w_z, float coeff, float v_x,float v_y, float v_z); 

    // void BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &body_vec);
    // void GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &global_vec);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    void getThrust(float timestamp, const euler_t& angles, FSMState FSM_state, Eigen::Vector3f& thrust_out);

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
   
    bool should_reinit = false;
    float current_vel = 0.0f;

private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    float Ca = 0;
    float Cn = 0;
    float Cp = 0;
    
    // Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
};

extern EKF ekf;
#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h" 
#include "constants.h"
#include "aero_coeff.h"
#include "rotation.h"

#define NUM_STATES 6  // [x, vx, y, vy, z, vz] - position and velocity only
#define NUM_SENSOR_INPUTS 4  // barometer, gps_x, gps_y, gps_z
#define NUM_CONTROL_INPUTS 3  // acceleration as control input [ax, ay, az]
#define ALTITUDE_BUFFER_SIZE 10


// Number of entries for aerodynamic data table
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    // void priori();
    void priori(float dt, Orientation &orientation, FSMState fsm, Acceleration acceleration); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state, GPS &gps) override;

    void setQ(float dt, float sd);
    void setF(float dt);
    void setB(float dt);  // Set control input matrix for acceleration 

    KalmanData getState() override;
    void setState(KalmanState state) override;
    void reference_GPS(GPS &gps, FSMState fsm); 

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state, GPS &gps);
   
    bool should_reinit = false;
    float current_vel = 0.0f;


private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    float Ca = 0;
    float Cn = 0;
    float Wind_alpha = 0.85f;
    float Cp = 0;
    float curr_mass_kg = mass_full; //(kg) Sustainer + Booster, but value changes over time.
    std::vector<float> starting_gps;    // latitude, longitude, altitude
    std::vector<float> starting_ecef;   // x, y, z

    // Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;
    
    // Track how long we've been in LANDED state to avoid false positives
    float landed_state_duration = 0.0f;
    bool was_landed_last = false;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
    
    // Control input matrix for acceleration [ax, ay, az]
    Eigen::Matrix<float, NUM_STATES, NUM_CONTROL_INPUTS> B_mat;
    
    // Last computed control input (acceleration) - filled by priori(), used by update() for state output
    Eigen::Matrix<float, NUM_CONTROL_INPUTS, 1> u_control;
    
    // GPS reference coordinates
    float gps_latitude_original = 0.0f;
    float gps_longitude_original = 0.0f;
    float gps_latitude_last = 0.0f;
    float gps_longitude_last = 0.0f;
};



extern EKF ekf;
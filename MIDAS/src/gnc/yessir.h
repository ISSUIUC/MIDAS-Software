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

    void setQ(float dt, float sd); // sets process noise covar. matrix, (dt = time step, sd = standard deviation of process noise)
    void setF(float dt); // sets state transition matrix, dt 
    Eigen::Matrix<float, 3, 1> BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> x_k);
    // transitions body vectors in respect to a global frame like earths surface
    // this takes roll, pitch, yaw ([3,1] vector) in body frame and returns it into global frame

    KalmanData getState() override; //gets current state of rocket
    void setState(KalmanState state) override; // updates/reinitializes kalman filter state, just to make sure its current

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
    /*  tick function updates prediction step and corrects state using sensor inputs by integrating the
        sensor measurements and system dynamics to refine state estimate
        dt = time step (change in time)
        sd = standard deviation
        baro -> gives altitude data
        accel -> gives accel data (usually used with baro inputs to update)
        orient -> probably uses angular positions to see how it is oriented and help account for system altitudes
        state -> represents current finite state machine of rocket and make sure it is correctly 
        updating based on phase (launch, coast, descent, etc.)

        ...
        just noticed that it defines all this when i hover the function lol but yeah this does the KF calculations
    
    */
   
    bool should_reinit = false; // makes sure to not restart system after calculating data (self-explanatory)
private:
    float s_dt = 0.05f; // 50ms time step
    float spectral_density_ = 13.0f; // spectral density is how much "weight" the process noise has (how noisy model is) 
    float kalman_apo = 0; //aposteriori error? not really sure at this point
    KalmanState kalman_state; // stores current state of KF

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero(); // matrix of zeros for acceleration (xyz)
    Eigen::Matrix<float, 3, 1> world_accel; // initializes matrix for accel in world frame
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer; // buffer, it stores a set amount of data before letting oldest go to stay current
    KalmanData state; //initializes state variable that tracks, well state
};

extern Yessir yessir;
// says that varaible is defined elsewhere (yessir.cpp) but just that
 
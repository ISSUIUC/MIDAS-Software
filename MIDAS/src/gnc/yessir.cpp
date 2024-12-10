#include "yessir.h"
#include "finite-state-machines/fsm_states.h"

Yessir::Yessir() : KalmanFilter() {
    state = KalmanData();
}

/**
 * @brief Sets altitude by averaging 30 barometer measurements taken 100 ms
 * apart
 *
 * The following for loop takes a series of barometer measurements on start
 * up and takes the average of them in order to initialize the kalman filter
 * to the correct initial barometric altitude. This is done so that the
 * kalman filter takes minimal time to converge to an accurate state
 * estimate. This process is significantly faster than allowing the state as
 * letting the filter to converge to the correct state can take up to 3 min.
 * This specific process was used because the barometric altitude will
 * change depending on the weather and thus, the initial state estimate
 * cannot be hard coded. A GPS altitude may be used instead but due to GPS
 * losses during high speed/high altitude flight, it is inadvisable with the
 * current hardware to use this as a solution. Reference frames should also
 * be kept consistent (do not mix GPS altitude and barometric).
 *
 */

void Yessir::initialize(RocketSystems* args) {
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();
    // gets most recent orientation data, doesn't care if it is being modified at the time of fetching it
    
    float sum = 0; // this one is used for baro altitude?????/
    
    for (int i = 0; i < 30; i++) {
        Barometer barometer = args->rocket_data.barometer.getRecent(); // gets recent barometer data, waits to be settled before fetching
        LowGData initial_accelerometer = args->rocket_data.low_g.getRecent(); //gets recent lowg accel data (prob a matrix?)
        Acceleration accelerations = { // accelerometer data put into xyz planes
            .ax = initial_accelerometer.ax, 
            .ay = initial_accelerometer.ay,
            .az = initial_accelerometer.az
        };
        sum += barometer.altitude; // altitude added to sum

        init_accel(0, 0) += accelerations.az; 
        init_accel(1, 0) += accelerations.ay; // matrix elements appended with accelerometer data
        init_accel(2, 0) += -accelerations.ax;
        THREAD_SLEEP(100);
        // pauses execution for 100ms before working with data again
    }

    init_accel(0, 0) /= 30; // remember the original loop went to thirty iterations
    init_accel(1, 0) /= 30; // this gets average accel for that instance from the
    init_accel(2, 0) /= 30; // 30 values added to it

    euler_t euler = orientation.getEuler(); // gets euler angles (row, pitch, yaw)
    euler.yaw = -euler.yaw; // not exactly sure, but probably a calculation thing
    world_accel = BodyToGlobal(euler, init_accel);
    // takes acceleration values in init_accel in xyz and based on the row,pitch,yaw angles, it converts to aa worldview accel from ground

    // set x_k
    x_k(0, 0) = sum / 30; // average altitude, refer to loop
    x_k(3, 0) = 0; // velocity
    x_k(6, 0) = 0; // accel, both start at 0 for are calculations to start

    // set F (state transition matrix)
    for (int i = 0; i < 3; i++) {                                           
        F_mat(3 * i, 3 * i + 1) = s_dt;                                     
        F_mat(3 * i, 3 * i + 2) = (s_dt * s_dt) / 2;                        
        F_mat(3 * i + 1, 3 * i + 2) = s_dt;                                 

        F_mat(3 * i, 3 * i) = 1;    // puts one across the diagonals
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;

        /* for visual purposes to understand, here is the completed matrix in terms of variables (for simplicity, da = dt^2/2)
        [[  1, dt, da,  0,  0,  0,  0,  0,  0],
         [  0,  1, dt, da,  0,  0,  0,  0,  0],
         [  0,  0,  1, dt, da,  0,  0,  0,  0],
         [  0,  0,  0,  1, dt, da,  0,  0,  0],
         [  0,  0,  0,  0,  1, dt, da,  0,  0],
         [  0,  0,  0,  0,  0,  1, dt, da,  0],
         [  0,  0,  0,  0,  0,  0,  1, dt, da],
         [  0,  0,  0,  0,  0,  0,  0,  1, dt],
         [  0,  0,  0,  0,  0,  0,  0,  0,  1]]
        */
    }
    // Process Noise Covariance Matrix
    Q(0, 0) = pow(s_dt, 5) / 20;
    Q(0, 1) = pow(s_dt, 4) / 8;
    Q(0, 2) = pow(s_dt, 3) / 6;
    Q(1, 1) = pow(s_dt, 3) / 8;
    Q(1, 2) = pow(s_dt, 2) / 2;
    Q(2, 2) = s_dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);

    Q(3, 3) = pow(s_dt, 5) / 20;
    Q(3, 4) = pow(s_dt, 4) / 8;
    Q(3, 5) = pow(s_dt, 3) / 6;
    Q(4, 4) = pow(s_dt, 3) / 8;
    Q(4, 5) = pow(s_dt, 2) / 2;
    Q(5, 5) = s_dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(s_dt, 5) / 20;
    Q(6, 7) = pow(s_dt, 4) / 8;
    Q(6, 8) = pow(s_dt, 3) / 6;
    Q(7, 7) = pow(s_dt, 3) / 8;
    Q(7, 8) = pow(s_dt, 2) / 2;
    Q(8, 8) = s_dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    /* same 9x9 matrix here for visuals
    [[ Qxx, Qxv, Qxa,   0,   0,   0,   0,   0,   0] Qxx = uncertainty in position itself                                    = s^5/20
     [ Qvx, Qvv, Qva,   0,   0,   0,   0,   0,   0] Qxv = how much velocity uncertainty will affect position uncertainty    = s^4/8
     [ Qax, Qav, Qaa,   0,   0,   0,   0,   0,   0] Qxa = how much accel uncertainty will affect position uncertainty       = s^3/6
     [   0,   0,   0, Qxx, Qxv, Qxa,   0,   0,   0] ------------------------------------------------------------------- 
     [   0,   0,   0, Qvx, Qvv, Qva,   0,   0,   0] Qvx = how much position uncertainty will affect velocity uncertainty    = s^4/8
     [   0,   0,   0, Qax, Qav, Qaa,   0,   0,   0] Qvv = uncertainty in velocity itself                                    = s^3/3
     [   0,   0,   0,   0,   0,   0, Qxx, Qxv, Qxa] Qva = how much accel uncertainty will affect velocity uncertainty       = s^2/2
     [   0,   0,   0,   0,   0,   0, Qvx, Qvv, Qva] -------------------------------------------------------------------
     [   0,   0,   0,   0,   0,   0, Qax, Qav, Qaa] Qax = how much position uncertainty will affect accel uncertainty       = s^3/6
                                                    Qav = how much velocity uncertainty will affect accel uncertainty       = s^2/2
                                                    Qaa = uncertainty in accel itself                                       = s

    values deal with integrals from pos vel accel equations, aint doin allat math but yeah thats why theres numbers like s^5/20
    */

    // set H (Observation Matrix) -> vector of measurements = (observation matrix)*vector + measurement noise
    H(0, 0) = 1; // [ 1, 0, 0, 0, 0, 0, 0, 0, 0]
    H(1, 2) = 1; // [ 0, 0, 1, 0, 0, 0, 0, 0, 0]
    H(2, 5) = 1; // [ 0, 0, 0, 0, 0, 1, 0, 0, 0]
    H(3, 8) = 1; // [ 0, 0, 0, 0, 0, 0, 0, 0, 1]

    Q = Q * spectral_density_; // mentioned before, header file says its 13.0
    // Q matrix is in unit variance, spectral density scales it

    // set R: Measurement Noise Covariance (lower = more reliable measurements)
    R(0, 0) = 2.0; // variance for barometric sensor
    R(1, 1) = 1.9; // variance for accelerometer
    R(2, 2) = 10; // variance of gyroscope
    R(3, 3) = 10; // varaince for state measurement

    // set B (don't care about what's in B since we have no control input)
    B(2, 0) = -1; // placeholder
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */
void Yessir::priori() {
     // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B
    // or u is = 0
    x_priori = (F_mat * x_k); // predicted state estimate matrix
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
    // predicted covariance matrix
}

/**
 * @brief Update Kalman Gain and state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate
 * and Kalman gain. The Kalman gain can be considered as a measure of how uncertain
 * the new sensor data is. After updating the gain, the state estimate is updated.
 *
 */
void Yessir::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state) {
// processes sensor data to adjust kf states
    if (FSM_state == FSMState::STATE_FIRST_BOOST || FSM_state == FSMState::STATE_SECOND_BOOST) { // boost phase of rocket
    // reads latest 10 altitude measurements from alt_buffer
    // find their average to initialize filter state
    // sets KF state vector with the avg altitude and 0 for other components to compute
        float sum = 0; // used for altitude, starts at 0
        float data[10]; // declares array of size 10
        alt_buffer.readSlice(data, 0, 10); // copies data from alt_buffer into data
        for (float i : data) {
            sum += i; // for each altitude reading in data, it adds to sum
        }
        KalmanState kalman_state = (KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0}; 
        // initializes with avg altitude
        setState(kalman_state); // updates filter internal state vector with new values we just got
    } else if (FSM_state >= FSMState::STATE_APOGEE) { // if phase after apogee
        H(1, 2) = 0; // this removes mapping between vel measurement and vel state
        // after apogee, velocity is not reliable to depend on for the rocket's state
    }

    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero(); 
    // S_k is the Innovation Covariance Matrix, represents uncertainty in the predicted measurement
    // it combines uncertainty from predicted state (P_priori) and the measurement noise (R)
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    // H = measurement matrix
    // R = measurement noise covariance
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity(); // identity matrix lol
    K = (P_priori * H.transpose()) * S_k;
    // Kalman gain matrix

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> accel = Eigen::Matrix<float, 3, 1>::Zero();
    // accel matrix
    accel(0, 0) = acceleration.az - 0.045; // constant offsets to account for
    accel(1, 0) = acceleration.ay - 0.065; // sensor biases or calibration adjustments
    accel(2, 0) = -acceleration.ax - 0.06;

    euler_t angles = orientation.getEuler(); // gyroscope data
    angles.yaw = -angles.yaw; // not exactly sure why

    Eigen::Matrix<float, 3, 1> acc = BodyToGlobal(angles, accel);
    // changes accel to earth view using gyroscope angle positions

    y_k(1, 0) = (acc(0)) * 9.81 - 9.81;
    y_k(2, 0) = (acc(1)) * 9.81;
    y_k(3, 0) = (acc(2)) * 9.81;

    y_k(0, 0) = barometer.altitude;
    alt_buffer.push(barometer.altitude);


    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;
    // Joseph (Expanded) Form
    // P_k = (identity - K * H) * P_priori * (identity - K * H).transpose() + K * R * K.transpose(); 
    
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

    state.position = (Position){kalman_state.state_est_pos_x,kalman_state.state_est_pos_y,kalman_state.state_est_pos_z};
    state.velocity = (Velocity){kalman_state.state_est_vel_x,kalman_state.state_est_vel_y,kalman_state.state_est_vel_z};
    state.acceleration = (Acceleration){kalman_state.state_est_accel_x,kalman_state.state_est_accel_y,kalman_state.state_est_accel_z};
}

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 * @param &barometer Data of the current barometer
 * @param acceleration Current acceleration
 * @param &orientation Current orientation
 * @param current_state Current FSM_state
 * 
 * you ahve to be kidding me.......
 */
void Yessir::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state) {
    if (FSM_state >= FSMState::STATE_IDLE) {
        setF(dt / 1000); // converts to milliseconds by dividing by 1000
        setQ(dt / 1000, sd); // uncertainty -> sd
        priori(); // does prediction step of kalman filter, estimating next state based on current factors
        update(barometer, acceleration, orientation, FSM_state); //
    }
    // this is from header file, looks like it doesn't do the actual calculation but sets it up
}

/**
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vect Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
Eigen::Matrix<float, 3, 1> Yessir::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> body_vect) {
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 0., cos(angles.roll), -sin(angles.roll), 0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch), 0., 1., 0., -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0., sin(angles.yaw), cos(angles.yaw), 0., 0., 0., 1.;
    return yaw * pitch * body_vect;
}


/**
 * @brief Getter for state X
 *
 * @return the current state, see sensor_data.h for KalmanData
 */
KalmanData Yessir::getState() {
    return state;
}

/**
 * @brief Sets state vector x
 *
 * @param state Wanted state vector
 */
void Yessir::setState(KalmanState state) {
    this->state.position.px = state.state_est_pos_x;
    this->state.position.py = state.state_est_pos_y;
    this->state.position.pz = state.state_est_pos_z;
    this->state.acceleration.ax = state.state_est_accel_x;
    this->state.acceleration.ay = state.state_est_accel_y;
    this->state.acceleration.az = state.state_est_accel_z;
    this->state.velocity.vx = state.state_est_vel_x;
    this->state.velocity.vy = state.state_est_vel_y;
    this->state.velocity.vz = state.state_est_vel_z;
}

/**
 * @brief Sets the Q matrix given time step and spectral density.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 *
 * The Q matrix is the covariance matrix for the process noise and is
 * updated based on the time taken per cycle of the Kalman Filter Thread.
 */
void Yessir::setQ(float dt, float sd) {
    Q(0, 0) = pow(dt, 5) / 20;
    Q(0, 1) = pow(dt, 4) / 8;
    Q(0, 2) = pow(dt, 3) / 6;
    Q(1, 1) = pow(dt, 3) / 8;
    Q(1, 2) = pow(dt, 2) / 2;
    Q(2, 2) = dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);
    Q(3, 3) = pow(dt, 5) / 20;
    Q(3, 4) = pow(dt, 4) / 8;
    Q(3, 5) = pow(dt, 3) / 6;
    Q(4, 4) = pow(dt, 3) / 8;
    Q(4, 5) = pow(dt, 2) / 2;
    Q(5, 5) = dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(dt, 5) / 20;
    Q(6, 7) = pow(dt, 4) / 8;
    Q(6, 8) = pow(dt, 3) / 6;
    Q(7, 7) = pow(dt, 3) / 8;
    Q(7, 8) = pow(dt, 2) / 2;
    Q(8, 8) = dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    Q *= sd;
}

/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time.
 */
void Yessir::setF(float dt) {
    for (int i = 0; i < 3; i++) {
        F_mat(3 * i, 3 * i + 1) = s_dt;
        F_mat(3 * i, 3 * i + 2) = (dt * s_dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = s_dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }
}

Yessir yessir;

#include "ekf.h"
#include "fsm_states.h"
#include <iostream>


EKF::EKF() : KalmanFilter()
{
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
void EKF::initialize(RocketSystems *args)
{
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();
    float sum = 0;

    for (int i = 0; i < 30; i++)
    {
        Barometer barometer = args->rocket_data.barometer.getRecent();
        sum += barometer.altitude;
    }

    // set x_k - 6 states: [x, vx, y, vy, z, vz]
    x_k.setZero();
    x_k(0, 0) = sum / 30;  // initial altitude (x position)
    x_k(2, 0) = 0;  // y position
    x_k(4, 0) = 0;  // z position

    F_mat.setZero(); // Initialize with zeros
    B_mat.setZero(); // Initialize control input matrix

    setQ(s_dt, spectral_density_);
    H.setZero();
    H(0, 0) = 1;  // barometer measures x position (altitude)
    H(1, 0) = 1;  // GPS measures x position (altitude)
    H(2, 2) = 1;  // GPS measures y position (east)
    H(3, 4) = 1;  // GPS measures z position (north)

    P_k.setZero();
    P_k.block<2, 2>(0, 0) = Eigen::Matrix2f::Identity() * 1e-2f; // x block (pos,vel)
    P_k.block<2, 2>(2, 2) = Eigen::Matrix2f::Identity() * 1e-2f; // y block (pos,vel)
    P_k.block<2, 2>(4, 4) = Eigen::Matrix2f::Identity() * 1e-2f; // z block (pos,vel)

    // set Measurement Noise Matrix
    R(0, 0) = 1.9;  // barometer noise
    R(1, 1) = 4.0f;  // GPS altitude noise (lower trust)
    R(2, 2) = 3.0f; // GPS east noise 
    R(3, 3) = 3.0f; // GPS north noise 
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void EKF::priori(float dt, Orientation &orientation, FSMState fsm, Acceleration acceleration)
{
    setF(dt);
    setB(dt);
    // Compute control input at current time step
    Eigen::Matrix<float, 3, 1> sensor_accel_global_g = Eigen::Matrix<float, 3, 1>::Zero();
    sensor_accel_global_g(0, 0) = acceleration.ax + 0.045f;
    sensor_accel_global_g(1, 0) = acceleration.ay - 0.065f;
    sensor_accel_global_g(2, 0) = acceleration.az - 0.06f;
    // euler_t angles_rad = orientation.getEuler();
    // BodyToGlobal(angles_rad, sensor_accel_global_g);
    // Do not apply gravity if on pad
    float g_ms2 = (fsm > FSMState::STATE_IDLE) ? gravity_ms2 : 0.0f;
    u_control(0, 0) = sensor_accel_global_g(0, 0) * g_ms2;
    u_control(1, 0) = sensor_accel_global_g(1, 0) * g_ms2;
    u_control(2, 0) = sensor_accel_global_g(2, 0) * g_ms2;
    // Predict state at time t and covariance
    x_priori = F_mat * x_k + B_mat * u_control;
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

/**
 * @brief Update state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate.
 * Updates with barometer (always) and GPS (if available).
 *
 */
void EKF::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state, GPS &gps)
{
    // if on pad take last 10 barometer measurements for init state
    if (FSM_state == FSMState::STATE_IDLE)
    {
        float sum = 0;
        float data[10];
        alt_buffer.readSlice(data, 0, 10);
        for (float i : data)
        {
            sum += i;
        }
        KalmanState kalman_state = (KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0};
        setState(kalman_state);
    }

    // Update GPS reference point if in IDLE state (needed before GPS measurement update)
    reference_GPS(gps, FSM_state);

    // Check if GPS has valid fix
    bool gps_available = (gps.fix_type != 0 && gps.latitude != 0 && gps.longitude != 0);
    
    // Build measurement vector and select appropriate H and R matrices
    Eigen::Matrix<float, NUM_STATES, NUM_STATES> identity = Eigen::Matrix<float, NUM_STATES, NUM_STATES>::Identity();
    
    if (gps_available)
    {
        // GPS available: use full H matrix 
        float lat = gps.latitude / 1e7; // dividing by 1e7 to convert from int to float
        float lon = gps.longitude / 1e7;
        float alt = gps.altitude;
        // Convert GPS to ECEF
        std::vector<float> rocket_cords = gps_to_ecef(lat, lon, alt);
        std::vector<float> reference_cord = gps_to_ecef(gps_latitude_original, gps_longitude_original, 0);
        float gps_latitude_original_rad = gps_latitude_original * pi / 180;
        float gps_longitude_original_rad = gps_longitude_original * pi / 180;
        double dx = rocket_cords[0] - reference_cord[0];
        double dy = rocket_cords[1] - reference_cord[1];
        double dz = rocket_cords[2] - reference_cord[2];
        float east = -std::sin(gps_longitude_original_rad) * dx + std::cos(gps_longitude_original_rad) * dy;
        float north = -std::sin(gps_latitude_original_rad) * std::cos(gps_longitude_original_rad) * dx - std::sin(gps_latitude_original_rad) * std::sin(gps_longitude_original_rad) * dy + std::cos(gps_latitude_original_rad) * dz;
        // Build measurement vector
        Eigen::Matrix<float, 4, 1> y_combined;
        y_combined(0) = barometer.altitude; // barometer altitude
        y_combined(1) = alt; // GPS altitude
        y_combined(2) = east; // GPS east
        y_combined(3) = north; // GPS north
        // Innovation
        Eigen::Matrix<float, 4, 1> innovation = y_combined - H * x_priori;
        // Innovation covariance
        Eigen::Matrix<float, 4, 4> S = H * P_priori * H.transpose() + R;
        // Kalman gain
        Eigen::Matrix<float, NUM_STATES, 4> K = P_priori * H.transpose() * S.inverse();
        // Update state and covariance
        x_k = x_priori + K * innovation;
        P_k = (identity - K * H) * P_priori;

        gps_latitude_last = lat;
        gps_longitude_last = lon;
    }
    else
    {
        // GPS not available, use only barometer H matrix
        Eigen::Matrix<float, 1, NUM_STATES> H_baro = H.block<1, NUM_STATES>(0, 0);
        Eigen::Matrix<float, 1, 1> R_baro;
        R_baro(0, 0) = R(0, 0);
        // Build measurement vector: [baro_alt]
        Eigen::Matrix<float, 1, 1> y_baro;
        y_baro(0) = barometer.altitude;
        
        // Innovation
        Eigen::Matrix<float, 1, 1> innovation = y_baro - H_baro * x_priori;
        // Innovation covariance
        Eigen::Matrix<float, 1, 1> S = H_baro * P_priori * H_baro.transpose() + R_baro;
        // Kalman gain
        Eigen::Matrix<float, NUM_STATES, 1> K = P_priori * H_baro.transpose() / S(0, 0);
        // Update state and covariance
        x_k = x_priori + K * innovation;
        P_k = (identity - K * H_baro) * P_priori;
    }

    bool is_landed = (FSM_state == FSMState::STATE_LANDED);
    if (is_landed)
    {
        if (was_landed_last)
        {
            landed_state_duration += s_dt;  // Accumulate time in LANDED state
        }
        else
        {
            landed_state_duration = s_dt;  // Reset timer
        }
        
        // Only reset velocities if we've been landed for at least 0.5 seconds
        if (landed_state_duration >= 0.5f)
        {
            x_k(3, 0) = 0.0f;  // velocity y (vy) = 0 when landed
            x_k(5, 0) = 0.0f;  // velocity z (vz) = 0 when landed
        }
    }
    else
    {
        // Negate velocity z if it's negative (so negative becomes positive)
        if (x_k(5, 0) < 0)  // velocity z (vz)
        {
            x_k(5, 0) = -x_k(5, 0);  // Negate negative velocity z to make it positive
        }
    }

    // Update output state structure
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = u_control(0, 0);  
    kalman_state.state_est_pos_y = x_k(2, 0);
    kalman_state.state_est_vel_y = x_k(3, 0);
    kalman_state.state_est_accel_y = u_control(1, 0);  
    kalman_state.state_est_pos_z = x_k(4, 0);
    kalman_state.state_est_vel_z = x_k(5, 0);
    kalman_state.state_est_accel_z = u_control(2, 0);  
    state.position = (Position){kalman_state.state_est_pos_x, kalman_state.state_est_pos_y, kalman_state.state_est_pos_z};
    state.velocity = (Velocity){kalman_state.state_est_vel_x, kalman_state.state_est_vel_y, kalman_state.state_est_vel_z};
    state.acceleration = (Acceleration){kalman_state.state_est_accel_x, kalman_state.state_est_accel_y, kalman_state.state_est_accel_z};
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
 */
void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state, GPS &gps)
{

    if (FSM_state >= FSMState::STATE_IDLE) //
    {
        if (FSM_state != last_fsm)
        {
            stage_timestamp = 0;
            last_fsm = FSM_state;
            // Reset landed state tracking when FSM changes
            if (FSM_state != FSMState::STATE_LANDED)
            {
                landed_state_duration = 0.0f;
                was_landed_last = false;
            }
        }
        stage_timestamp += dt;
        s_dt = dt;  // Store dt for use in update
        
        setQ(dt, sd);
        priori(dt, orientation, FSM_state, acceleration);
        update(barometer, acceleration, orientation, FSM_state, gps);
    }
}

/**
 * @brief Getter for state X
 *
 * @return the current state, see sensor_data.h for KalmanData
 */
KalmanData EKF::getState()
{
    return state;
}

/**
 * @brief Sets state vector x
 *
 * @param state Wanted state vector
 */
void EKF::setState(KalmanState state)
{
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
void EKF::setQ(float dt, float sd)
{
    // discrete-time integrated acceleration noise
    float sigma_a = 0.2f; 
    Q.setZero();
    // X axis
    Q(0,0) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(0,1) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(1,0) = Q(0,1);
    Q(1,1) = pow(dt,2) * sigma_a*sigma_a;
    // Y axis
    Q(2,2) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(2,3) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(3,2) = Q(2,3);
    Q(3,3) = pow(dt,2) * sigma_a*sigma_a;
    // Z axis
    Q(4,4) = pow(dt,4)/4.0f * sigma_a*sigma_a;
    Q(4,5) = pow(dt,3)/2.0f * sigma_a*sigma_a;
    Q(5,4) = Q(4,5);
    Q(5,5) = pow(dt,2) * sigma_a*sigma_a;
}

/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time and also depends on the
 * current state of the rocket.
 */
void EKF::setF(float dt)
{
    F_mat.setIdentity();
    F_mat(0, 1) = dt;  // x += vx * dt
    F_mat(2, 3) = dt;  // y += vy * dt
    F_mat(4, 5) = dt;  // z += vz * dt
}

void EKF::setB(float dt)
{
    B_mat.setZero();
    B_mat(0, 0) = 0.5f*dt*dt; // x += 1/2 * ax * dt^2
    B_mat(1, 0) = dt;  // vx += ax * dt
    B_mat(2, 1) = 0.5f*dt*dt; // y += 1/2 * ay * dt^2
    B_mat(3, 1) = dt;  // vy += ay * dt
    B_mat(4, 2) = 0.5f*dt*dt; // z += 1/2 * az * dt^2
    B_mat(5, 2) = dt;  // vz += az * dt
}

void EKF::reference_GPS(GPS &gps, FSMState fsm)
{
    if (gps.latitude == 0 || gps.longitude == 0)
    {
        return; // No GPS fix skip reference update
    }

    if (fsm == FSMState::STATE_IDLE)
    {
        gps_latitude_original = gps.latitude / 1e7;
        gps_longitude_original = gps.longitude / 1e7;
        gps_latitude_last = gps_latitude_original;
        gps_longitude_last = gps_longitude_original;
    }
}

EKF ekf;
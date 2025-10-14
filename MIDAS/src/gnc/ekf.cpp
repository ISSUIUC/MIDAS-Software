#include "ekf.h"
#include "finite-state-machines/fsm_states.h"

extern const std::map<float, float> O5500X_data;
extern const std::map<float, float> M685W_data;
extern const std::map<std::string, std::map<float, float>> motor_data;

EKF::EKF() : KalmanFilter()
{
    state = KalmanData();
}

/**
 * THIS IS A PLACEHOLDER FUNCTION SO WE CAN ABSTRACT FROM `kalman_filter.h`
 */
void EKF::priori() {};

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
        LowGData initial_accelerometer = args->rocket_data.low_g.getRecent();
        Acceleration accelerations = {
            .ax = initial_accelerometer.ax,
            .ay = initial_accelerometer.ay,
            .az = initial_accelerometer.az};
        sum += barometer.altitude;

        init_accel(0, 0) += accelerations.az;
        init_accel(1, 0) += accelerations.ay;
        init_accel(2, 0) += -accelerations.ax;
        THREAD_SLEEP(100);
    }

    init_accel(0, 0) /= 30;
    init_accel(1, 0) /= 30;
    init_accel(2, 0) /= 30;

    euler_t euler = orientation.getEuler();
    euler.yaw = -euler.yaw;

    // set x_k
    x_k(0, 0) = sum / 30;
    x_k(3, 0) = 0;
    x_k(6, 0) = 0;

    F_mat.setZero(); // Initialize with zeros

    // Initialize Q from filterpy
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

    // set H
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 5) = 1;
    H(3, 8) = 1;

    Q = Q * spectral_density_;

    // set R
    R(0, 0) = 2.0;
    R(1, 1) = 1.9;
    R(2, 2) = 10;
    R(3, 3) = 10;

    // set B (don't care about what's in B since we have no control input)
    B(2, 0) = -1;
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void EKF::priori(float dt, Orientation &orientation, FSMState fsm)
{
    Eigen::Matrix<float, 9, 1> xdot = Eigen::Matrix<float, 9, 1>::Zero();


    // angular states from sensors
    Velocity omega_rps = orientation.getVelocity(); // rads per sec

    euler_t angles_rad = orientation.getEuler();

    // ignore effects of gravity when on pad
    Eigen::Matrix<float,3,1> Fg_global = Eigen::Matrix<float,3,1>::Zero();
    if ((fsm > FSMState::STATE_IDLE) && (fsm < FSMState::STATE_LANDED))
    {
        Fg_global(0, 0) = -gravity_ms2;
    }
    else
    {
        Fg_global(0, 0) = 0;
    }

    // mass and height init
    float curr_mass_kg = mass_sustainer;
    float curr_height_m = height_sustainer;

    if (fsm < FSMState::STATE_BURNOUT)
    {
        curr_mass_kg = mass_full;
        curr_height_m = height_full;
    }

    // Mach number
    float vel_mag_squared_ms = x_k(1, 0) * x_k(1, 0) + x_k(4, 0) * x_k(4, 0) + x_k(7, 0) * x_k(7, 0);

    float vel_magnitude_ms = pow(vel_mag_squared_ms, 0.5);

    float mach = vel_magnitude_ms / a;


    // approximating C_a (aerodynamic coeff.)
    int index = std::round(mach / 0.04);

    index = std::clamp(index, 0, (int)AERO_DATA_SIZE - 1);


    Ca = aero_data[index].CA_power_on;

    // aerodynamic force
    // Body frame
    float Fax = -0.5 * rho * (vel_mag_squared_ms) * float(Ca) * (pi * r * r);
    float Fay = 0; // assuming no aerodynamic effects
    float Faz = 0; // assuming no aerodynamic effects

    Serial.println("vel_mag_squared_ms: " + String(vel_mag_squared_ms));


    

    // force due to gravity
    Eigen::Matrix<float, 3, 1> Fg_body = Fg_global;

    GlobalToBody(angles_rad, Fg_body);

    float Agx = Fg_body(0, 0);
    float Agy = Fg_body(1, 0);
    float Agz = Fg_body(2, 0);

    // thurst force
    Eigen::Matrix<float, 3, 1> Ft_global;
    EKF::getThrust(stage_timestamp, angles_rad, fsm, Ft_global);

    // Global frame
    float Ftx = Ft_global(0, 0);
    float Fty = Ft_global(1, 0);
    float Ftz = Ft_global(2, 0); 


    xdot << x_k(1, 0),
        ((Fax + Ftx) / curr_mass_kg + Agx - (omega_rps.vy * x_k(7, 0) - omega_rps.vz * x_k(4, 0)) + x_k(2, 0)) * 0.5,
        0.0,

        x_k(4, 0),
        ((Fay + Fty) / curr_mass_kg + Agy - (omega_rps.vz * x_k(1, 0) - omega_rps.vx * x_k(7, 0)) + x_k(5, 0)) * 0.5,
        0.0,

        x_k(7, 0),
        ((Faz + Ftz) / curr_mass_kg + Agz - (omega_rps.vx * x_k(4, 0) - omega_rps.vy * x_k(1, 0)) + x_k(8, 0)) * 0.5,
        0.0;
        
    // priori step
    x_priori = (xdot * dt) + x_k;
    setF(dt, omega_rps.vx, omega_rps.vy, omega_rps.vz);

    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;

}

/**
 * @brief Update Kalman Gain and state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate
 * and Kalman gain. The Kalman gain can be considered as a measure of how uncertain
 * the new sensor data is. After updating the gain, the state estimate is updated.
 *
 */
void EKF::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state)
{
    // if on pad -> take last 10 barometer measurements for init state
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
    // ignore alitiude measurements after apogee
    else if (FSM_state >= FSMState::STATE_APOGEE)
    {
        H(1, 2) = 0;
    }

    // Kalman Gain
    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * S_k;

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> sensor_accel_global_g = Eigen::Matrix<float, 3, 1>(Eigen::Matrix<float, 3, 1>::Zero());

    // accouting for sensor bias and coordinate frame transforms
    (sensor_accel_global_g)(0, 0) = acceleration.ax - 0.045;
    (sensor_accel_global_g)(1, 0) = acceleration.ay - 0.065;
    (sensor_accel_global_g)(2, 0) = acceleration.az - 0.06;

    euler_t angles_rad = orientation.getEuler();
    angles_rad.yaw = -angles_rad.yaw; // coordinate frame match

    BodyToGlobal(angles_rad, sensor_accel_global_g);

    float g_ms2;
    if ((FSM_state > FSMState::STATE_IDLE) && (FSM_state < FSMState::STATE_LANDED))
    {
        g_ms2 = -gravity_ms2;
    }
    else
    {
        g_ms2 = 0;
    }

    // acceloremeter reports values in g's and measures specific force
    y_k(1, 0) = ((sensor_accel_global_g)(0)) * gravity_ms2;
    y_k(2, 0) = ((sensor_accel_global_g)(1)) * gravity_ms2;
    y_k(3, 0) = ((sensor_accel_global_g)(2)) * gravity_ms2;

    y_k(0, 0) = barometer.altitude; // meters

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

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
void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state)
{
    if (FSM_state >= FSMState::STATE_IDLE)
    {
        if (FSM_state != last_fsm)
        {
            stage_timestamp = 0;
            last_fsm = FSM_state;
        }
        stage_timestamp += dt;

        setF(dt, orientation.roll, orientation.pitch, orientation.yaw);
        setQ(dt, sd);
        priori(dt, orientation, FSM_state); 
        update(barometer, acceleration, orientation, FSM_state);
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
void EKF::
    setQ(float dt, float sd)
{
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
 * by how the states change over time and also depends on the
 * current state of the rocket.
 */
void EKF::setF(float dt, float wx, float wy, float wz)
{
    F_mat(0, 1) = 1;
    F_mat(1, 2) = 0.5;
    F_mat(1, 4) = wz * 0.5;
    F_mat(1, 7) = -wy * 0.5;
    F_mat(3, 4) = 1;
    F_mat(4, 1) = -wz * 0.5;
    F_mat(4, 5) = 0.5;
    F_mat(4, 7) = wx * 0.5;
    F_mat(6, 7) = 1;
    F_mat(7, 1) = wy * 0.5;
    F_mat(7, 4) = -wx * 0.5;
    F_mat(7, 8) = 0.5;
}

/**
 * @brief Returns the approximate thrust force from the motor given the thurst curve
 *
 * @param timestamp Time since most recent ignition
 * @param angles Current orientation of the rocket
 * @param FSM_state Current FSM state
 *
 * @return Thrust force in the body frame
 *
 * The thrust force is calculated by interpolating the thrust curve data which is stored in an ordered map (see top of file).
 * The thrust curve data is different for the booster and sustainer stages, so the function checks the FSM state to determine
 * which thrust curve to use. The time since ignition is also important to consider so that is reset once we reach a new stage.
 * The thrust force is then rotated into the body frame using the BodyToGlobal function.
 */
void EKF::getThrust(float timestamp, const euler_t& angles, FSMState FSM_state, Eigen::Vector3f& thrust_out)
{
    // Pick which motor thrust curve to use
    const std::map<float, float>* thrust_curve = nullptr;
    if (FSM_state >= STATE_FIRST_BOOST && FSM_state < FSMState::STATE_BURNOUT)
        thrust_curve = &motor_data.at("Booster"); // Booster
    else if (FSM_state >= FSMState::STATE_BURNOUT)
        thrust_curve = &motor_data.at("Sustainer");  // Sustainer
    else {
        thrust_out.setZero();
        return; // No thrust before ignition
    }

    // Handle case where timestamp is before or after available data
    if (timestamp <= thrust_curve->begin()->first) {
        thrust_out = Eigen::Vector3f(thrust_curve->begin()->second, 0.f, 0.f);
    }
    else if (timestamp >= thrust_curve->rbegin()->first) {
        thrust_out.setZero(); // assume motor burned out after curve ends
    }
    else {
        // Find interpolation interval
        auto it_upper = thrust_curve->lower_bound(timestamp);
        auto it_lower = std::prev(it_upper);

        float x0 = it_lower->first;
        float y0 = it_lower->second;
        float x1 = it_upper->first;
        float y1 = it_upper->second;

        float interpolated_thrust = linearInterpolation(x0, y0, x1, y1, timestamp);
        thrust_out = Eigen::Vector3f(interpolated_thrust, 0.f, 0.f);
    }

    // Rotate from body to global
    BodyToGlobal(angles, thrust_out);
}

EKF ekf;

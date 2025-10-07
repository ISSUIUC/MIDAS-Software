#include "ekf.h"
#include "finite-state-machines/fsm_states.h"

extern const std::map<float, float> O5500X_data;
extern const std::map<float, float> M685W_data;

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
    Velocity omega = orientation.getVelocity();
    euler_t angles = orientation.getEuler();
    // Eigen::Matrix<float, 3, 1> gravity = Eigen::Matrix<float, 3,1>::Zero();
    if ((fsm > FSMState::STATE_IDLE) && (fsm < FSMState::STATE_LANDED))
    {
        gravity(0, 0) = -grav_ms;
    }
    else
    {
        gravity(0, 0) = 0;
    }

    // mass and height initialization
    float curr_mass_kg = mass_sustainer;
    float curr_height_m = height_sustainer;

    if (fsm < FSMState::STATE_BURNOUT)
    {
        curr_mass_kg = mass_full;
        curr_height_m = height_full;
    }

    float w_x = omega.vx;
    float w_y = omega.vy;
    float w_z = omega.vz;

    // finding the Mach number
    float vel_mag_squared_ms = x_k(1, 0) * x_k(1, 0) + x_k(4, 0) * x_k(4, 0) + x_k(7, 0) * x_k(7, 0);
    float vel_magnitude_ms = pow(vel_mag_squared_ms, 0.5);
    float mach = vel_magnitude_ms / a;

    // finding the closest aerodynamic coeff. C_a
    int index = std::round(mach / 0.04);
    index = std::clamp(index, 0, (int)AERO_DATA_SIZE - 1);
    Ca = aero_data[index].CA_power_on;

    // aerodynamic force
    float Fax = -0.5 * rho * (vel_mag_squared_ms) * float(Ca) * (pi * r * r);
    float Fay = 0; // assuming no aerodynamic effects
    float Faz = 0; // assuming no aerodynamic effects

    // force due to gravity
    Eigen::Matrix<float, 3, 1> Fg_body;
    EKF::GlobalToBody(angles, Fg_body);

    float Fgx = gravity(0, 0);
    float Fgy = gravity(1, 0);
    float Fgz = gravity(2, 0);

    // thurst force
    Eigen::Matrix<float, 3, 1> thrust_vec;

    EKF::getThrust(stage_timestamp, angles, fsm, thrust_vec);

    float Ftx = thrust_vec(0, 0);
    float Fty = thrust_vec(1, 0);
    float Ftz = thrust_vec(2, 0);

    xdot << x_k(1, 0),
        ((Fax + Ftx + Fgx) / curr_mass_kg - (w_y * x_k(7, 0) - w_z * x_k(4, 0)) + x_k(2, 0)) * 0.5,
        0.0,

        x_k(4, 0),
        ((Fay + Fty + Fgy) / curr_mass_kg - (w_z * x_k(1, 0) - w_x * x_k(7, 0)) + x_k(5, 0)) * 0.5,
        0.0,

        x_k(7, 0),
        ((Faz + Ftz + Fgz) / curr_mass_kg - (w_x * x_k(4, 0) - w_y * x_k(1, 0)) + x_k(8, 0)) * 0.5,
        0.0;
        
    // priori step
    x_priori = (xdot * dt) + x_k;
    setF(dt, w_x, w_y, w_z);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

/**
 * @brief linearly interpolates the a value based on the lower and upper bound, similar to lerp_() in PySim
 */
float EKF::linearInterpolation(float x0, float y0, float x1, float y1, float x)
{
    return y0 + ((x - x0) * (y1 - y0) / (x1 - x0));
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
void EKF::getThrust(float timestamp, euler_t angles, FSMState FSM_state, Eigen::Matrix<float, 3, 1> &to_modify)
{
    float interpolatedValue = 0;
    if (FSM_state >= STATE_FIRST_BOOST)
    {
        if (FSM_state < FSMState::STATE_BURNOUT)
        {
            // first stage
            if (timestamp >= 0.009) // TODO: index the first value from the correct motor
            {
                auto it = O5500X_data.lower_bound(timestamp);
                if (it != O5500X_data.end())
                {
                    float x0 = it->first;
                    float y0 = it->second;
                    ++it;
                    float x1 = it->first;
                    float y1 = it->second;
                    interpolatedValue = linearInterpolation(x0, y0, x1, y1, timestamp);
                }
            }
        }
        else
        {
            if (timestamp >= 0.083) // TODO: index the first value from the correct motor
            {
                // second stage
                auto it = M685W_data.lower_bound(timestamp);
                if (it != M685W_data.end())
                {
                    float x0 = it->first;
                    float y0 = it->second;
                    ++it;
                    float x1 = it->first;
                    float y1 = it->second;
                    interpolatedValue = linearInterpolation(x0, y0, x1, y1, timestamp);
                }
            }
        }
    }
    Eigen::Matrix<float, 3, 1> interpolatedVector = Eigen::Matrix<float, 3, 1>::Zero();
    (interpolatedVector)(0, 0) = interpolatedValue;
    EKF::BodyToGlobal(angles, interpolatedVector, to_modify);
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
    else if (FSM_state >= FSMState::STATE_APOGEE)
    {
        H(1, 2) = 0;
    }

    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * S_k;

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> accel = Eigen::Matrix<float, 3, 1>(Eigen::Matrix<float, 3, 1>::Zero());

    // TODO: document this right
    (accel)(0, 0) = acceleration.az - 0.045;
    (accel)(1, 0) = acceleration.ay - 0.065;
    (accel)(2, 0) = -acceleration.ax - 0.06;

    euler_t angles = orientation.getEuler();
    angles.yaw = -angles.yaw;

    Eigen::Matrix<float, 3, 1> acc;
    EKF::BodyToGlobal(angles, accel, acc);

    float g;
    if ((FSM_state > FSMState::STATE_IDLE) && (FSM_state < FSMState::STATE_LANDED))
    {
        g = -grav_ms;
    }
    else
    {
        g = 0;
    }

    // TODO: magic numbers
    y_k(1, 0) = ((acc)(0)) * 9.81 + g;
    y_k(2, 0) = ((acc)(1)) * 9.81;
    y_k(3, 0) = ((acc)(2)) * 9.81;

    y_k(0, 0) = barometer.altitude;
    alt_buffer.push(barometer.altitude);

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
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vect Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
void EKF::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &body_vect, Eigen::Matrix<float, 3, 1> &to_modify)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 0., cos(angles.roll), -sin(angles.roll), 0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch), 0., 1., 0., -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0., sin(angles.yaw), cos(angles.yaw), 0., 0., 0., 1.;

    to_modify = yaw * pitch * roll * (body_vect);
}

/**
 * THIS IS A PLACEHOLDER FUNCTION SO WE CAN ABSTRACT FROM `kalman_filter.h`
 */
void EKF::priori() {};

/**
 * @brief Converts a vector in the global frame to the body frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param world_vector Vector for rotation in the global frame
 *
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the body frame
 * TODO: Don't pass in gravity and pass in a vector instead
 */
void EKF::GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &to_modify)
{
    Eigen::Matrix3f roll;
    roll << 1, 0, 0, 0, cos(angles.roll), -sin(angles.roll), 0, sin(angles.roll), cos(angles.roll);
    Eigen::Matrix3f pitch;
    pitch << cos(angles.pitch), 0, sin(angles.pitch), 0, 1, 0, -sin(angles.pitch), 0, cos(angles.pitch);
    Eigen::Matrix3f yaw;
    yaw << cos(angles.yaw), -sin(angles.yaw), 0, sin(angles.yaw), cos(angles.yaw), 0, 0, 0, 1;
    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    to_modify = rotation_matrix.transpose() * gravity;
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

EKF ekf;

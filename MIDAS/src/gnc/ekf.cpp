#include "ekf.h"
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
    
    float sum = 0;
    
    for (int i = 0; i < 30; i++) {
        Barometer barometer = args->rocket_data.barometer.getRecent();
        LowGData initial_accelerometer = args->rocket_data.low_g.getRecent();
        Acceleration accelerations = {
            .ax = initial_accelerometer.ax,
            .ay = initial_accelerometer.ay,
            .az = initial_accelerometer.az
        };
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
    world_accel = BodyToGlobal(euler, init_accel);

    // set x_k
    x_k(0, 0) = sum / 30;
    x_k(3, 0) = 0;
    x_k(6, 0) = 0;

    // set F
    for (int i = 0; i < 3; i++) {
        F_mat(3 * i, 3 * i + 1) = s_dt;
        F_mat(3 * i, 3 * i + 2) = (s_dt * s_dt) / 2;
        F_mat(3 * i + 1, 3 * i + 2) = s_dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }

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

    //once FSM > 9, after its LANDED
    std::map<double, double> moonburner_data = {
        {0.083, 1333.469},
        {0.13, 1368.376},
        {0.249, 1361.395},
        {0.308, 1380.012},
        {0.403, 1359.068},
        {0.675, 1184.53},
        {1.018, 1072.826},
        {1.456, 996.029},
        {1.977, 958.794},
        {2.995, 914.578},
        {3.99, 856.399},
        {4.985, 781.929},
        {5.494, 730.732},
        {5.991, 679.534},
        {7.258, 542.231},
        {7.862, 463.107},
        {8.015, 456.125},
        {8.998, 330.458},
        {9.993, 207.118},
        {10.514, 137.303},
        {11.496, 34.908},
        {11.994, 0.0}
    };

    //before FSM = 9 
    std::map<double, double> O5500X_data = {
        {0.009, 20.408},
        {0.044, 7112.245},
        {0.063, 6734.694},
        {0.078, 6897.959},
        {0.094, 6612.245},
        {0.109, 6765.306},
        {0.125, 6540.816},
        {0.147, 6581.633},
        {0.194, 6520.408},
        {0.35, 6795.918},
        {0.428, 7091.837},
        {0.563, 7285.714},
        {0.694, 7408.163},
        {0.988, 7581.633},
        {1.266, 7622.449},
        {1.491, 7724.49},
        {1.581, 7653.061},
        {1.641, 7540.816},
        {1.684, 7500.0},
        {1.716, 7336.735},
        {1.784, 7224.49},
        {1.938, 6785.714},
        {2.138, 6326.531},
        {2.491, 5897.959},
        {2.6, 5704.082},
        {2.919, 3540.816},
        {3.022, 3408.163},
        {3.138, 2887.755},
        {3.3, 2234.694},
        {3.388, 1673.469},
        {3.441, 1489.796},
        {3.544, 1418.367},
        {3.609, 1295.918},
        {3.688, 816.327},
        {3.778, 653.061},
        {3.819, 581.633},
        {3.853, 489.796},
        {3.897, 285.714},
        {3.981, 20.408},
        {3.997, 0.0}
    };

}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

Eigen::Matrix<float, 3, 1> Yessir::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> body_vect) {
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 0., cos(angles.roll), -sin(angles.roll), 0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch), 0., 1., 0., -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0., sin(angles.yaw), cos(angles.yaw), 0., 0., 0., 1.;
    return yaw * pitch * roll * body_vect;
}

void Yessir::priori(float dt, float m, float r, float h) {
    dt = dt / 1000;
    Eigen::Matrix<float, 9, 9> xdot = Eigen::Matrix<float, 9, 9>::Zero();

    Orientation orientation = args->rocket_data.orientation.getRecent();

    euler_t angles = orientation.getEuler();
    Eigen::Matrix<float, 3,1> gravity = Eigen::Matrix<float, 3,1>::Zero();
    gravity(2, 0) = -9.81;
    world_accel = BodyToGlobal(angles, init_accel) + gravity;

    float w_x = world_accel(0, 0) * dt;
    float w_y = world_accel(1, 0) * dt;
    float w_z = world_accel(2, 0) * dt;

    float J_x = 1/2 * m * r**2
    float J_y = 1/3 * m * h**2 + 1/4 * m * r**2
    float J_z = J_y;

    float Fax = 0;
    float Fay = 0;
    float Faz = 0;

//  Fax = -0.5*rho*(vel_mag**2)*float(Ca)*(np.pi*r**2)

    Fg_body = R.inverse() * gravity; 
    
    Fgx = Fg_body[0];
    Fgy = Fg_body[1];
    Fgz = Fg_body[2];    
    T = getThrust();

    xdot << vel_x,
            (Fax + Ftx + Fgx) / m - (w_y * vel_z - w_z * vel_y),
            1.0,

            vel_y,
            (Fay + Fty + Fgy) / m - (w_z * vel_x - w_x * vel_y),
            1.0,

            vel_z,
            (Faz + Ftz + Fgz) / m - (w_x * vel_y - w_y * vel_x),
            1.0;

    for (int i = 0; i < 3; i++) {
        xdot(3 * i, 3 * i + 1) = xdot();
        xdot(3 * i, 3 * i + 2) = (s_dt * s_dt) / 2;
        xdot(3 * i + 1, 3 * i + 2) = s_dt;

        F_mat(3 * i, 3 * i) = 1;
        F_mat(3 * i + 1, 3 * i + 1) = 1;
        F_mat(3 * i + 2, 3 * i + 2) = 1;
    }

    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

Eigen::Matrix<float, 3, 1> Yessir:getThrust(euler_t angles, FSMState FSM_state) {
    Eigen::Matrix<float, 3,1> T;
    
    // interpolate thrust values (weighted avg)

    if(FSM > 9) {
        thrust(0, 0) = 0;
        thrust(1, 0) = 0;
        thrust(2, 0) = 0;
    } else if (FSM <= 9) {
        thrust(0, 0) = T * cos(angles.pitch) * sin(angles.roll);
        thrust(1, 0) = T * sin(angles.pitch);
        thrust(2, 0) = T * cos(angles.pitch) * cos(angles.roll);
    } 

    // return 3x1 thrust vector

    return T;
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
    if (FSM_state == FSMState::STATE_FIRST_BOOST || FSM_state == FSMState::STATE_SECOND_BOOST) { 
        float sum = 0;
        float data[10];
        alt_buffer.readSlice(data, 0, 10);
        for (float i : data) {
            sum += i;
        }
        KalmanState kalman_state = (KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0};
        setState(kalman_state);
    } else if (FSM_state >= FSMState::STATE_APOGEE) {
        H(1, 2) = 0;
    }

    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * S_k;

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> accel = Eigen::Matrix<float, 3, 1>::Zero();
    
    accel(0, 0) = acceleration.az - 0.045;
    accel(1, 0) = acceleration.ay - 0.065;
    accel(2, 0) = -acceleration.ax - 0.06;

    euler_t angles = orientation.getEuler();
    angles.yaw = -angles.yaw;

    Eigen::Matrix<float, 3, 1> acc = BodyToGlobal(angles, accel);

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
 */
void Yessir::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state) {
    if (FSM_state >= FSMState::STATE_IDLE) {
        setF(dt / 1000);
        setQ(dt / 1000, sd);
        priori();
        update(barometer, acceleration, orientation, FSM_state);
    }
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
    return yaw * pitch * roll * body_vect;
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

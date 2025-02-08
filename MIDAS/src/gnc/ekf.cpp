#include "ekf.h"
#include "finite-state-machines/fsm_states.h"


EKF::EKF() : KalmanFilter() {
    state = KalmanData();
}


// constants
const float pi = 3.14159268;
const float a = 343.0; // (m/s) speed of sound
const float rho = 1.225; // average air density
const float r = 0.03935; // (m)
const float height_full = 2.34; // (m) height of rocket Full Stage
const float height_sustainer = 1.34;  // (m) height of rocket Sustainer
const float mass_full = 7.57; // (kg) Sustainer + Booster
const float mass_sustainer = 4.08; // (kg) Sustainer

typedef struct {
    float mach;
    float alpha;
    float CA_power_on;
    float CN;
    float CP;
} AeroCoeff;

// stores the aerodynamic coefficients for the corresponding Mach number
const AeroCoeff aero_data[] = {
    {0.04, 4, 1.332142905, 1.1827808455, 60.8267871968},
    {0.08, 4, 1.326587387, 1.1827808455, 60.8267871968},
    {0.12, 4, 1.31558627,  1.1827808455, 60.8267871968},
    {0.16, 4, 1.306063953, 1.1827808455, 60.8267871968},
    {0.20, 4, 1.298117084, 1.1827808455, 60.8267871968},
    {0.24, 4, 1.291411025, 1.1827808455, 60.8267871968},
    {0.28, 4, 1.290279857, 1.1827808455, 60.8267871968},
    {0.32, 4, 1.291431043, 1.1827808455, 60.8267871968},
    {0.36, 4, 1.293170653, 1.1827808455, 60.8267871968},
    {0.40, 4, 1.295385827, 1.1827808455, 60.8267871968},
    {0.44, 4, 1.297991738, 1.1827808455, 60.8267871968},
    {0.48, 4, 1.300924032, 1.1827808455, 60.8267871968},
    {0.52, 4, 1.304132086, 1.1827808455, 60.8267871968},
    {0.56, 4, 1.309039395, 1.1827808455, 60.8267871968},
    {0.60, 4, 1.314605487, 1.1827808455, 60.8267871968},
    {0.64, 4, 1.330699437, 1.1827808455, 60.8267871968},
    {0.68, 4, 1.346695167, 1.1827808455, 60.8267871968},
    {0.72, 4, 1.362693183, 1.1827808455, 60.8267871968},
    {0.76, 4, 1.378693074, 1.1827808455, 60.8267871968},
    {0.80, 4, 1.394695194, 1.1827808455, 60.8267871968},
    {0.84, 4, 1.41069913,  1.1827808455, 60.8267871968},
    {0.88, 4, 1.426705046, 1.1827808455, 60.8267871968},
    {0.92, 4, 1.473732816, 1.218959468,  60.91848997},
    {0.96, 4, 1.582395672, 1.291316713,  61.10189551},
    {1.00, 4, 1.681494886, 1.363673958,  61.28530105},
};

// Number of entries
#define AERO_DATA_SIZE (sizeof(aero_data) / sizeof(aero_data[0]))



// Moonburner motor thrust curve (Sustainer)
const std::map<float, float> moonburner_data = {
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


// O5500X motor thrust curve (Booster)
std::map<float, float> O5500X_data = {
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


void EKF::initialize(RocketSystems* args) {
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

    // set x_k
    x_k(0, 0) = sum / 30;
    x_k(3, 0) = 0;
    x_k(6, 0) = 0;


    F_mat.setZero();  // Initialize with zeros


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


void EKF::priori(float dt, Orientation &orientation, FSMState fsm) {
    Eigen::Matrix<float, 9, 1> xdot = Eigen::Matrix<float, 9, 1>::Zero();
    Velocity omega = orientation.getVelocity();
    euler_t angles = orientation.getEuler();
    // Eigen::Matrix<float, 3, 1> gravity = Eigen::Matrix<float, 3,1>::Zero();
    gravity(0, 0) = -9.81;   
    float m = mass_sustainer; float h = height_sustainer;
    if (fsm < FSMState::STATE_BURNOUT) {
        m = mass_full; h = height_full;
    }
    float w_x = omega.vx; float w_y = omega.vy; float w_z = omega.vz;
   
    float J_x = 0.5 *  m * r * r;
    float J_y = (1/3) * m * h * h + 0.25 * m * r * r;
    float J_z = J_y;
   
    float vel_mag_squared = x_k(1,0)*x_k(1,0) + x_k(4,0)*x_k(4,0) + x_k(7,0)*x_k(7,0);


    float Fax = -0.5*rho*(vel_mag_squared)*float(Ca)*(pi*r*r);
    float Fay = 0; float Faz = 0;


    Eigen::Matrix<float, 3, 1> *Fg_body = (GlobalToBody(angles, gravity));
    float Fgx = (*Fg_body)(0,0); float Fgy = (*Fg_body)(1,0); float Fgz = (*Fg_body)(2,0);  

    delete Fg_body;

    Eigen::Matrix<float, 3, 1> * T = EKF::getThrust(stage_timestamp, angles, fsm);
    float Ftx = (*T)(0, 0); float Fty = (*T)(1, 0); float Ftz = (*T)(2, 0);

    delete T;

   
    xdot << x_k(1,0),
            (Fax + Ftx + Fgx) / m - (w_y * x_k(7,0) - w_z * x_k(4,0)),
            1.0,


            x_k(4,0),
            (Fay + Fty + Fgy) / m - (w_z * x_k(1, 0) - w_x * x_k(7,0)),
            1.0,


            x_k(7,0),
            (Faz + Ftz + Fgz) / m - (w_x * x_k(4,0) - w_y * x_k(1,0)),
            1.0;


    x_priori = (xdot * dt) + x_k;
    setF(dt, fsm, w_x, w_y, w_z);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}


/**
 * @brief linearly interpolates the a value based on the lower and upper bound, similar to lerp_() in PySim
 */
float EKF::linearInterpolation(float x0, float y0, float x1, float y1, float x) {
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
Eigen::Matrix<float, 3, 1> *EKF::getThrust(float timestamp, euler_t angles, FSMState FSM_state) {
    float interpolatedValue = 0;
    if (FSM_state >= STATE_FIRST_BOOST){
        if (FSM_state < FSMState::STATE_BURNOUT ) {
            // first stage
            if(timestamp >= 0.009){
                auto it = O5500X_data.lower_bound(timestamp);
                if (it != O5500X_data.end()) {
                    float x0 = it->first;
                    float y0 = it->second;
                    ++it;
                    float x1 = it->first;
                    float y1 = it->second;
                    interpolatedValue = linearInterpolation(x0, y0, x1, y1, timestamp);
                }
            }
        }  
        else {
            if (timestamp >= 0.083){
                // second stage
                auto it = moonburner_data.lower_bound(timestamp);
                if (it != moonburner_data.end()) {
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
    Eigen::Matrix<float, 3,1> interpolatedVector;
    interpolatedVector(0,0) = interpolatedValue;
    return BodyToGlobal(angles, interpolatedVector);
}


/**
 * @brief Update Kalman Gain and state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate
 * and Kalman gain. The Kalman gain can be considered as a measure of how uncertain
 * the new sensor data is. After updating the gain, the state estimate is updated.
 *
 */
void EKF::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state) {
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


    Eigen::Matrix<float, 3, 1> * acc = (BodyToGlobal(angles, accel));


    y_k(1, 0) = ((*acc)(0)) * 9.81 - 9.81;
    y_k(2, 0) = ((*acc)(1)) * 9.81;
    y_k(3, 0) = ((*acc)(2)) * 9.81;

    delete acc;

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
void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state) {
    if (FSM_state >= FSMState::STATE_IDLE) {
        if (FSM_state != last_fsm) {
            stage_timestamp = 0;
        }
        stage_timestamp += dt;
        setF(dt, FSM_state, orientation.roll, orientation.pitch, orientation.yaw);
        setQ(dt, sd);
        priori(dt, orientation, FSM_state);
        update(barometer, acceleration, orientation, FSM_state);
        FSMState last_fsm = FSM_state;
    }
}


/**
 * @brief Getter for state X
 *
 * @return the current state, see sensor_data.h for KalmanData
 */
KalmanData EKF::getState() {
    return state;
}


/**
 * @brief Sets state vector x
 *
 * @param state Wanted state vector
 */
void EKF::setState(KalmanState state) {
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
void EKF::setQ(float dt, float sd) {
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
Eigen::Matrix<float, 3, 1> * EKF::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> body_vect) {
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 0., cos(angles.roll), -sin(angles.roll), 0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch), 0., 1., 0., -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0., sin(angles.yaw), cos(angles.yaw), 0., 0., 0., 1.;
   
    Eigen::Matrix<float, 3, 1> * global = new Eigen::Matrix<float, 3, 1>(yaw * pitch * roll * body_vect);
    return global;
}


/**
 * THIS IS A PLACEHOLDER FUNCTION SO WE CAN ABSTRACT FROM `kalman_filter.h`
 */
void EKF::priori(){};


/**
 * @brief Converts a vector in the global frame to the body frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param world_vector Vector for rotation in the global frame
 *
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the body frame
 *
 */
Eigen::Matrix<float, 3, 1> * EKF::GlobalToBody(euler_t angles, const Eigen::Matrix<float, 3, 1> world_vector) {
   
    Eigen::Matrix3f roll;
    roll << 1, 0, 0, 0, cos(angles.roll), -sin(angles.roll), 0, sin(angles.roll), cos(angles.roll);
    Eigen::Matrix3f pitch;
    pitch << cos(angles.pitch), 0, sin(angles.pitch), 0, 1, 0,-sin(angles.pitch), 0, cos(angles.pitch);
    Eigen::Matrix3f yaw;
    yaw << cos(angles.yaw), -sin(angles.yaw), 0, sin(angles.yaw), cos(angles.yaw), 0, 0, 0, 1;
    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    (rotation_matrix).transpose();
    Eigen::Matrix<float, 3, 1> * to_return = new Eigen::Matrix<float,3,1>(rotation_matrix*world_vector);
    return to_return;

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
void EKF::setF(float dt, FSMState fsm, float wx, float wy, float wz) {
    Eigen::Matrix<float, 3, 1> w = Eigen::Matrix<float, 3, 1>::Zero();
    w(0, 0) = wx;
    w(1, 0) = wy;
    w(2, 0) = wz;
    F_mat(0,1) = 1;
    F_mat(3,4) = 1;
    F_mat(6,7) = 1;

    
    float velocity_magnitude = pow(x_k(1,0)*x_k(1,0) + x_k(4,0)*x_k(4,0) + x_k(7,0)*x_k(7,0), 0.5);
    float mach = velocity_magnitude / a;
    int index = std::round(mach / 0.04);
    index = std::clamp(index, 0, (int)AERO_DATA_SIZE - 1);

    Ca = aero_data[index].CA_power_on;
    Cn = aero_data[index].CN;
    Cp = aero_data[index].CP;

    float m = mass_sustainer;
    float h = height_sustainer;
    if (fsm < FSMState::STATE_BURNOUT) {
        m = mass_full;  
        h= height_full;
    }


    F_mat(1,1) = -pi * Ca * r * r * rho * x_k(1,0) / m;
    F_mat(1,4) = -pi * Ca * r * r * rho * x_k(4,0) / m + w(2,0);
    F_mat(1,7) = -pi * Ca * r * r * rho * x_k(7,0) / m - w(1,0);


    F_mat(4,1) = pi * Cn * r * r * rho * x_k(1,0) / m - w(2,0);
    F_mat(4,4) = pi * Cn * r * r * rho * x_k(2,0) / m + w(0,0);
    F_mat(4,7) = pi * Cn * r * r * rho * x_k(3,0) / m;


    F_mat(7,1) = pi * Cn * r * r * rho * x_k(1,0) / m + w(1,0);
    F_mat(7,4) = pi * Cn * r * r * rho * x_k(2,0) / m - w(0,0);
    F_mat(7,7) = pi * Cn * r * r * rho * x_k(3,0) / m;
}

EKF ekf;




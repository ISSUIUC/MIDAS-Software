#include "yessir.h"
#include "src/finite-state-machines/fsm_states.h"


Yessir::Yessir() : KalmanFilter() {
    state = KalmanData();
}

void Yessir::initialize(Orientation &orientation, Barometer &barometer, Acceleration &acceleration) {
    // TODO: The altitude initialization is the same code as
    //   setLaunchPadElevation() in AC. Maybe use the same one?
    float sum = 0;
    
    for (int i = 0; i < 30; i++) {
        // TODO This mutex lock is almost certainly not necessary
        //chMtxLock(&barometer.mutex);
        sum += barometer.altitude;
        //chMtxUnlock(&barometer.mutex);

        //chMtxLock(&highG.mutex);
        init_accel(0, 0) += acceleration.az;
        init_accel(1, 0) += acceleration.ay;
        init_accel(2, 0) += -acceleration.ax;
        //chMtxUnlock(&highG.mutex);
        //chThdSleepMilliseconds(100);
    }

    init_accel(0, 0) /= 30;
    init_accel(1, 0) /= 30;
    init_accel(2, 0) /= 30;

    //chMtxLock(&orientation.mutex);
    euler_t euler = orientation.getEuler();
    //chMtxUnlock(&orientation.mutex);
    euler.yaw = -euler.yaw;
    world_accel = BodyToGlobal(euler, init_accel);

    // set x_k
    x_k(0, 0) = sum / 30;
    // x_k(0,0) = 1401;
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

    float spectral_density = 13.0;
    Q = Q * spectral_density;

    // set R
    R(0, 0) = 2.0;
    R(1, 1) = 1.9;
    R(2, 2) = 10;
    R(3, 3) = 10;

    // set B (don't care about what's in B since we have no control input)
    B(2, 0) = -1;
}

void Yessir::priori() {
     // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B
    // or u is = 0
    x_priori = (F_mat * x_k);
    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}

void Yessir::update(Barometer barometer, Acceleration acceleration, Orientation orientation) {
    // Ask correct state
    if (getActiveFSM().getFSMState() == FSM_State::STATE_LAUNCH_DETECT) { 
        float sum = 0;
        float data[10];
        alt_buffer.readSlice(data, 0, 10);
        for (float i : data) {
            sum += i;
        }
        setState((KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    } else if (getActiveFSM().getFSMState() >= FSMState::STATE_APOGEE) {
        H(1, 2) = 0;
    }

    Eigen::Matrix<float, 4, 4> temp = Eigen::Matrix<float, 4, 4>::Zero();
    temp = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * temp;

    // TODO These mutex locks are almost certainly not necessary
    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> accel = Eigen::Matrix<float, 3, 1>::Zero();
    //chMtxLock(&highG.mutex);
    accel(0, 0) = acceleration.az - 0.045;
    accel(1, 0) = acceleration.ay - 0.065;
    accel(2, 0) = -acceleration.ax - 0.06;
    //chMtxUnlock(&highG.mutex);
    //chMtxLock(&orientation.mutex);
    euler_t angles = orientation.getEuler();
    // euler_t angles = (euler_t){0, 0, 0};
    //chMtxUnlock(&orientation.mutex);
    angles.yaw = -angles.yaw;

    Eigen::Matrix<float, 3, 1> acc = BodyToGlobal(angles, accel);

    y_k(1, 0) = (acc(0)) * 9.81 - 9.81;
    // Serial.println(y_k(1, 0));
    y_k(2, 0) = (acc(1)) * 9.81;
    y_k(3, 0) = (acc(2)) * 9.81;

    //chMtxLock(&barometer.mutex);
    y_k(0, 0) = barometer.altitude;
    alt_buffer.push(barometer.altitude);
    //chMtxUnlock(&barometer.mutex);

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    //chMtxLock(&mutex);
    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

    //timestamp = chVTGetSystemTime();
    //chMtxUnlock(&mutex);

    Position kalman_state_position = {{kalman_state.state_est_pos_x},{kalman_state.state_est_pos_y},{kalman_state.state_est_pos_z}};
    Velocity kalman_state_velocity = {{kalman_state.state_est_vel_x},{kalman_state.state_est_vel_y},{kalman_state.state_est_vel_z}};
    Acceleration kalman_state_acceleration = {{kalman_state.state_est_accel_x},{kalman_state.state_est_accel_y},{kalman_state.state_est_accel_z}};
    state.position = kalman_state_position;
    state.velocity = kalman_state_velocity;
    state.acceleration = kalman_state_acceleration;
    state.altitude = kalman_apo;
    //kalman_data.timeStamp_state = timestamp;

    //dataLogger.pushKalmanFifo(kalman_data); Do I need this?
}

void Yessir::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation) {
    if (getActiveFSM().getFSMState() >= FSMState::STATE_IDLE) {
        setF(float(dt) / 1000);
        setQ(float(dt) / 1000, sd);
        priori();
        update(barometer, acceleration, orientation);
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
    return yaw * pitch * body_vect;
}

KalmanData Yessir::getState() {
    return state;
}

void Yessir::setState(KalmanState state) {
    this->state.position.px = state.state_est_pos_x;
    this->state.position.py = state.state_est_pos_y;
    this->state.position.pz = state.state_est_pos_z;
    this->state.acceleration.ax = state.state_est_accel_x;
    this->state.acceleration.ay = state.state_est_accel_y;
    this->state.acceleration.az = state.state_est_accel_z;
    this->state.velocity.vx =state.state_est_vel_x;
    this->state.velocity.vy =state.state_est_vel_y;
    this->state.velocity.vz =state.state_est_vel_z;
}

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

/**
 * @brief Sets the apogee estimate
 *
 * @param estimate Apogee estimate
 */
//Ask about this
void Yessir::updateApogee(float estimate) { kalman_apo = estimate; }

Yessir yessir;

#include "mqekf.h"
#include "sensor_data.h"

void QuaternionMEKF::initialize(RocketSystems *args)
{
    Eigen::Matrix<float, 3, 1> sigma_a = {300 * sqrt(100.0f) * 1e-6 * 9.81, 300 * sqrt(100.0f) * 1e-6 * 9.81, 300 * sqrt(100.0f) * 1e-6 * 9.81}; // ug/sqrt(Hz) *sqrt(hz). values are from datasheet
    Eigen::Matrix<float, 3, 1> sigma_g = {0.03 / sqrt(3) * M_PI / 180, 0.03 / sqrt(3) * M_PI / 180, 0.03 / sqrt(3) * M_PI / 180};                                                 // 0.1 deg/s
    Eigen::Matrix<float, 3, 1> sigma_m = {-3.2e-4 / sqrt(3), 3.2e-4 / sqrt(3), 4.1e-4 / sqrt(3)};

    float Pq0 = 1e-6;
    float Pb0 = 1e-1;
    Q = initialize_Q(sigma_g);

    Eigen::Matrix<float, 6, 1> sigmas;
    sigmas << sigma_a, sigma_m;
    R = sigmas.array().square().matrix().asDiagonal();

    qref.setIdentity(); // 1,0,0,0
    x.setZero();
    P.setZero();
    P.block<3, 3>(0, 0) = Pq0 * Eigen::Matrix3f::Identity();
    P.block<3, 3>(3, 3) = Pb0 * Eigen::Matrix3f::Identity();

    Acceleration accel, accel_sum;
    Magnetometer mag, mag_sum;
    FSMState FSM_state = arg->rocket_data.fsm_state.getRecent();
    if (FSM_state == FSMState::STATE_IDLE)
    {
        for (int i = 0; i < 10; i++)
        {
            accel = args->rocket_data.imu.highg_acceleration.getRecent();
            accel_sum += accel;
            mag = args->rocket_data.magnetometer.getRecent();
            mag_sum += mag;
        }
        accel = accel_sum /= 10;
        else
        {
            accel = args->rocket_data.imu.highg_acceleration.getRecent();
            mag = args->rocket_data.magnetometer.getRecent();
        }

        // MagnetometerSensor mag = mag_sum / 10; // args->rocket_data.magnetometer.getRecentUnsync();
        // Acceleration accel = accel_sum / 10;   // args->rocket_data.imu.Velocity.getRecent();
        initialize_from_acc_mag(accel, mag);
    }
}

QuaternionMEKF::QuaternionMEKF()
{
    state = AngularKalmanData();
}

void QuaternionMEKF::tick(float dt, Magnetometer &magnetometer, Velocity &angular_velocity, Acceleration &acceleration, FSMState FSM_state)
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
        s_dt = dt; // Store dt for use in update

        // setQ(dt, sd);
        // priori(dt, orientation, FSM_state, acceleration);
        // update(barometer, acceleration, orientation, FSM_state, gps);

        time_update(angular_velocity, dt)
        measurement_update(acceleration, magnetometer);
        Eigen<float,4,1> curr_quat = quat(); // w,x,y,z

        state.quat.w = curr_quat(0,0);
        state.quat.x = curr_quat(1,0);
        state.quat.y = curr_quat(2,0);
        state.quat.z = curr_quat(3,0);
        state.has_data = true; // not sure what this is

        Eigen::Matrix<float,3,1> orientation= quatToEuler(quat); 
        state.roll = orientation(0,0);
        state.pitch = orientation(1,0);
        state.yaw = orientation(2,0);
        
    }
}

void QuaternionMEKF::time_update(Velocity const &gyro, float Ts)
{
    // Conversion from degrees/s to radians/s

    Eigen::Matrix<float, 3, 1> gyr;
    gyr(0, 0) = gyro.vx * (M_PI / 180.0f);
    gyr(1, 0) = gyro.vy * (M_PI / 180.0f);
    gyr(2, 0) = gyro.vz * (M_PI / 180.0f);

    set_transition_matrix(gyr - x.tail(3), Ts);

    Eigen::Vector4f q; // necessary to reorder to w,x,y,z
    q << qref.w(), qref.x(), qref.y(), qref.z();

    q = F * q;

    qref = Eigen::Quaternionf(q(0), q(1), q(2), q(3));
    qref.normalize();

    Eigen::Matrix<float, 6, 6> F_a;
    // Slice 3x3 block from F
    F_a << F.block(0, 0, 3, 3), (-Eigen::Matrix<float, 3, 3>::Identity() * Ts),
        Eigen::Matrix<float, 3, 3>::Zero(), Eigen::Matrix<float, 3, 3>::Identity();
    P = F_a * P * F_a.transpose() + Q; // P update
}

void QuaternionMEKF::measurement_update(Acceleration const &accel, Magnetometer const &magn)
{
    // accel measurements
    Eigen::Matrix<float, 3, 1> acc;
    acc(0, 0) = accel.ax;
    acc(1, 0) = accel.ay;
    acc(2, 0) = accel.az;

    Eigen::Matrix<float, 3, 1> mag;
    mag(0, 0) = magn.mx;
    mag(1, 0) = magn.my;
    mag(2, 0) = magm.mz;

    Eigen::Matrix<float, 3, 1> const v1hat = accelerometer_measurement_func();
    Eigen::Matrix<float, 3, 1> const v2hat = magnetometer_measurement_func();

    Eigen::Matrix<float, 3, 3> const C1 = skew_symmetric_matrix(v1hat);
    Eigen::Matrix<float, 3, 3> const C2 = skew_symmetric_matrix(v2hat);

    Eigen::Matrix<float, 6, 6> C;
    C << C1, Eigen::Matrix<float, 3, 3>::Zero(),
        C2, Eigen::Matrix<float, 3, 3>::Zero();

    Eigen::Matrix<float, 6, 1> yhat;
    yhat << v1hat,
        v2hat;

    Eigen::Matrix<float, 6, 1> y;
    y << acc,
        mag;

    Eigen::Matrix<float, 6, 1> inno = y - yhat;

    Eigen::Matrix<float, 6, 6> const s = C * P * C.transpose() + R;

    // K = P * C.float *(s)^-1
    // K * s = P*C.float

    // This is the form
    // x * A = b
    // Which can be solved with the code below
    Eigen::FullPivLU<Eigen::Matrix<float, 6, 6>> lu(s); //  LU decomposition of s
    if (lu.isInvertible())
    {
        Eigen::Matrix<float, 6, 6> const K = P * C.transpose() * lu.inverse(); // gain

        x += K * inno; // applying correction???

        // Joseph form of covariance measurement update
        Eigen::Matrix<float, 6, 6> const temp = Eigen::Matrix<float, 6, 6>::Identity() - K * C;
        P = temp * P * temp.transpose() + K * R * K.transpose(); // covariance update???
        // Apply correction to qref
        Eigen::Quaternion<float> corr(1, 0.5f * x(0), 0.5f * x(1), 0.5f * x(2)); // small angle approx????
        corr.normalize();
        qref = qref * corr; // multiply quaternions from ref???????

        // We only want to reset the quaternion part of the state
        x(0) = 0;
        x(1) = 0;
        x(2) = 0;
    }
}

void QuaternionMEKF::measurement_update_partial(
    Eigen::Matrix<float, 3, 1> const &meas,
    Eigen::Ref<Eigen::Matrix<float, 3, 1> const> const &vhat,
    Eigen::Ref<Eigen::Matrix<float, 3, 3> const> const &Rm)
{
    // Predicted measurement Jacobian
    Eigen::Matrix<float, 3, 3> const C1 = skew_symmetric_matrix(vhat);

    Eigen::Matrix<float, 3, 6> C;

    C << C1, Eigen::Matrix<float, 3, 3>::Zero();

    // Innovation
    Eigen::Matrix<float, 3, 1> const inno = meas - vhat;

    // Innovation covariance
    Eigen::Matrix<float, 3, 3> const s = C * P * C.transpose() + Rm;

    // K = P * C.T * s^-1
    Eigen::FullPivLU<Eigen::Matrix<float, 3, 3>> lu(s);
    if (lu.isInvertible())
    {
        Eigen::Matrix<float, 6, 3> const K = P * C.transpose() * lu.inverse();

        // State update
        x += K * inno;

        // Joseph form covariance update
        Eigen::Matrix<float, 6, 6> const temp =
            Eigen::Matrix<float, 6, 6>::Identity() - K * C;
        P = temp * P * temp.transpose() + K * Rm * K.transpose();

        // Apply correction to reference quaternion (small-angle approx)
        Eigen::Quaternion<float> corr(
            1.0f,
            0.5f * x(0),
            0.5f * x(1),
            0.5f * x(2));
        corr.normalize();
        qref = qref * corr;

        // Reset attitude error states
        x(0) = 0.0f;
        x(1) = 0.0f;
        x(2) = 0.0f;
    }
}

Eigen::Matrix<float, 4, 1> QuaternionMEKF::quaternion()
{
    return qref.coeffs();
}

void QuaternionMEKF::set_transition_matrix(Eigen::Ref<const Eigen::Matrix<float, 3, 1>> const &gyr, float Ts)
{
    Eigen::Matrix<float, 3, 1> const delta_theta = gyr * Ts;
    float un = delta_theta.norm();
    if (un == 0)
        un = std::numeric_limits<float>::min();

    Eigen::Matrix<float, 4, 4> const Omega = (Eigen::Matrix<float, 4, 4>() << -skew_symmetric_matrix(delta_theta), delta_theta,
                                              -delta_theta.transpose(), 0)
                                                 .finished();

    F = std::cos(0.5f * un) * Eigen::Matrix<float, 4, 4>::Identity() + std::sin(0.5f * un) / un * Omega;
}

Eigen::Matrix<float, 3, 3> QuaternionMEKF::skew_symmetric_matrix(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>> &vec) const
{
    Eigen::Matrix<float, 3, 3> M;
    M << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;

    return M;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::accelerometer_measurement_func() const
{
    return qref.inverse() * v1ref;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::magnetometer_measurement_func() const
{
    return qref.inverse() * v2ref;
}

Eigen::Matrix<float, 6, 6> QuaternionMEKF::initialize_Q(Eigen::Matrix<float, 3, 1> sigma_g)
{
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
    Q.block<3, 3>(0, 0) = sigma_g.array().square().matrix().asDiagonal();
    Q.block<3, 3>(3, 3) = 1e-12 * Eigen::Matrix3f::Identity();
    return Q;
}

void QuaternionMEKF::initialize_from_acc_mag(Eigen::Matrix<float, 3, 1> const &acc, Eigen::Matrix<float, 3, 1> const &mag)
{
    float const anorm = acc.norm();
    v1ref << anorm, 0, 0;

    Eigen::Matrix<float, 3, 1> const acc_normalized = acc / anorm;
    Eigen::Matrix<float, 3, 1> const mag_normalized = mag.normalized();

    Eigen::Matrix<float, 3, 1> const Rz = -acc_normalized;
    Eigen::Matrix<float, 3, 1> const Ry = Rz.cross(mag_normalized).normalized();
    Eigen::Matrix<float, 3, 1> const Rx = Ry.cross(Rz).normalized();

    // Construct the rotation matrix
    Eigen::Matrix<float, 3, 3> const R = (Eigen::Matrix<float, 3, 3>() << Rx, Ry, Rz).finished();

    // Eigen can convert it to a quaternion
    qref = R.transpose();

    // Reference magnetic field vector
    v2ref = qref * mag;
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::gyroscope_bias()
{
    return x.tail(3);
}

Eigen::Matrix<float, 3, 1> QuaternionMEKF::quatToEuler(const Eigen::Matrix<float, 4, 1> &q)
{

    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return Eigen::Matrix<float, 3, 1>(roll, pitch, yaw);
}

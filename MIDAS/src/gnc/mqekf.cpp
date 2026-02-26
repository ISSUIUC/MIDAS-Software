#include "mqekf.h"

/*
    mag
    y -> -x
    x -> y
    z -> -z

    acc
    pitch -> roll
    roll -> pitch
*/

void QuaternionMEKF::initialize(RocketSystems *args)
{
    float Pq0 = 1e-6;
    float Pb0 = 1e-1;
    sigma_a = {accel_noise_density_x * sqrt(100.0f) * 1e-6 * 9.81, accel_noise_density_y * sqrt(100.0f) * 1e-6 * 9.81, accel_noise_density_z * sqrt(100.0f) * 1e-6 * 9.81}; // ug/sqrt(Hz) *sqrt(hz). values are from datasheet
    sigma_g = {0.1 * pi / 180, 0.1 * pi / 180, 0.1 * pi / 180};                                                                                                             // 0.1 deg/s
    sigma_m = {0.4e-4 / sqrt(3), 0.4e-4 / sqrt(3), 0.4e-4 / sqrt(3)};                                                                                                       // 0.4 mG -> T, it is 0.4 total so we divide by sqrt3
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
    FSMState FSM_state = args->rocket_data.fsm_state.getRecent();
    if (FSM_state == FSMState::STATE_IDLE)
    {
        for (int i = 0; i < 10; i++)
        {
            IMU imu_data = args->rocket_data.imu.getRecent();

            Acceleration accel = imu_data.highg_acceleration;
            accel_sum.ax += accel.ax;
            accel_sum.ay += accel.ay;
            accel_sum.az += accel.az;

            mag = args->rocket_data.magnetometer.getRecent();

            mag_sum.mx += mag.mx;
            mag_sum.my += mag.my;
            mag_sum.mz += mag.mz;
        }
        accel.ax = accel_sum.ax / 10;
        accel.ay = accel_sum.ay / 10;
        accel.az = accel_sum.az / 10;

        mag.mx = mag_sum.mx / 10;
        mag.my = mag_sum.my / 10;
        mag.mz = mag_sum.mz / 10;
    }
    else
    {
        IMU imu_data = args->rocket_data.imu.getRecent();
        accel = imu_data.highg_acceleration;
        mag = args->rocket_data.magnetometer.getRecent();
    }

    // MagnetometerSensor mag = mag_sum / 10; // args->rocket_data.magnetometer.getRecentUnsync();
    // Acceleration accel = accel_sum / 10;   // args->rocket_data.imu.Velocity.getRecent();
    initialize_from_acc_mag(accel, mag);
}

QuaternionMEKF::QuaternionMEKF()
{
    state = AngularKalmanData();
}

void QuaternionMEKF::tick(float dt, Magnetometer &magnetometer, Velocity &angular_velocity, Acceleration &acceleration, FSMState FSM_state)
{
    if (FSM_state >= FSMState::STATE_IDLE) //
    {

        // setQ(dt, sd);
        // priori(dt, orientation, FSM_state, acceleration);
        // update(barometer, acceleration, orientation, FSM_state, gps);

        time_update(angular_velocity, dt);
        measurement_update(acceleration, magnetometer);
        Eigen::Matrix<float, 4, 1> curr_quat = quaternion(); // w,x,y,z

        state.quaternion.w = curr_quat(0, 0);
        state.quaternion.x = curr_quat(1, 0);
        state.quaternion.y = curr_quat(2, 0);
        state.quaternion.z = curr_quat(3, 0);
        state.has_data = true; // not sure what this is

        Eigen::Matrix<float, 3, 1> orientation = quatToEuler(curr_quat);
        state.roll = orientation(0, 0);
        state.pitch = orientation(1, 0);
        state.yaw = orientation(2, 0);

        Eigen::Matrix<float, 3, 1> bias_gyro = gyroscope_bias();
        state.gyrobias[0] = bias_gyro(0, 0);
        state.gyrobias[1] = bias_gyro(1, 0);
        state.gyrobias[2] = bias_gyro(2, 0);
    }
}

void QuaternionMEKF::time_update(Velocity const &gyro, float Ts)
{
    // Conversion from degrees/s to radians/s

    Eigen::Matrix<float, 3, 1> gyr;
    gyr(0, 0) = gyro.vx * (pi / 180.0f);
    gyr(1, 0) = gyro.vy * (pi / 180.0f);
    gyr(2, 0) = gyro.vz * (pi / 180.0f);

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

void QuaternionMEKF::measurement_update(Acceleration const &accel, Magnetometer const &mag_input)
{
    // accel measurements
    Eigen::Matrix<float, 3, 1> acc;
    acc(0, 0) = accel.ax;
    acc(1, 0) = accel.ay;
    acc(2, 0) = accel.az;

    Eigen::Matrix<float, 3, 1> mag;
    mag(0, 0) = mag_input.mx;
    mag(1, 0) = mag_input.my;
    mag(2, 0) = mag_input.mz;

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
    if (lu.determinant() != 0)                                 // change from invertible to allow tiny determinant to pass through 
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

void QuaternionMEKF::initialize_from_acc_mag(Acceleration const &acc_struct, Magnetometer const &mag_struct)
{
    Eigen::Matrix<float, 3, 1> acc;
    acc << acc_struct.ax, acc_struct.ay, acc_struct.az;
    Eigen::Matrix<float, 3, 1> mag;
    mag << mag_struct.mx, mag_struct.my, mag_struct.mz;
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
        pitch = std::copysign(pi / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return Eigen::Matrix<float, 3, 1>(roll, pitch, yaw);
}

AngularKalmanData QuaternionMEKF::getState()
{
    return state;
}

void QuaternionMEKF::calculate_tilt()
{

    const float alpha = 0.98; // Higher values dampen out current measurements --> reduce peaks

    // The guess & check method!
    // Quat --> euler --> rotation matrix --> reference&cur vector --> dot product for angle!

    Eigen::Quaternion<float>
        ref = Eigen::Quaternionf(1, 0, 0, 0);

    Eigen::Quaternion<float> rotated = qref * ref * qref.conjugate();

    Eigen::Matrix<float, 1, 3> reference_vector = {0, 0, -1};
    Eigen::Matrix<float, 1, 3> rotated_vector = {rotated.x(), rotated.y(), rotated.z()};

    float dot = rotated_vector.dot(reference_vector);
    float cur_mag = rotated_vector.norm();
    float ref_mag = reference_vector.norm();

    float tilt = 0;
    if (cur_mag != 0 && ref_mag != 0)
    {
        tilt = acos(dot / (cur_mag * ref_mag));
    }

    const float gain = 0.2;
    // Arthur's Comp Filter
    float filtered_tilt = gain * tilt + (1 - gain) * prev_tilt;
    prev_tilt = filtered_tilt;
    state.mq_tilt = filtered_tilt;

    // Serial.print("TILT: ");
    // Serial.println(filtered_tilt * (180/3.14f));
}

QuaternionMEKF mqekf;
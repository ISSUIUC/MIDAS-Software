#include <cmath>

#include <Eigen/Eigen>
#include <Adafruit_BNO08x.h>

#include "sensors.h"
#include "pins.h"

// global static instance of the sensor
Adafruit_BNO08x imu(BNO086_RESET);
#define REPORT_INTERVAL_US 5000
// 25 ms
#define REPORT_INTERVAL_US_AV 100000
unsigned long lastTime = 0;
float deltaTime = 0;
sh2_SensorValue_t event;

/**
 * @brief Initializes the bno sensor
 *
 * @return Error Code
 */
ErrorCode init_orientation() {
    digitalWrite(BNO086_RESET, OUTPUT);
    delay(100);
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!imu.begin_SPI(BNO086_CS, BNO086_INT)) {
        return ErrorCode::CannotConnectBNO;
    }
    Serial.println("Setting BNO desired reports");
    if (!imu.enableReport(SH2_ARVR_STABILIZED_RV, REPORT_INTERVAL_US)) {
        return ErrorCode::CannotInitBNO;
    }

    if (!imu.enableReport(SH2_GYRO_INTEGRATED_RV, REPORT_INTERVAL_US_AV)) {
        return ErrorCode::CannotInitBNO;
    }

    return ErrorCode::NoError;
}

float angle_between_quaternions(const Quaternion& q1, const Quaternion& q2) {
    float dot_product = Quaternion::dot(q1, q2);
    return 2.0f * std::acos(std::fabs(dot_product));
}

/**
 * @brief Turns a quaternion into its corresponding Euler 3D vector representation
 *
 * @param qr Quaternion real component
 * @param qi Quaternion i component
 * @param qj Quaternion j component
 * @param qk Quaternion k component
 * @param degrees Quaternion degrees, not used
 *
 * @return 3D representation of the quaternion
 */
Vec3 quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    Vec3 euler;
    euler.x = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));       // roll
    euler.y = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));      // yaw
    euler.z = -1 * atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)); // pitch
    return euler;
}

/**
 * @brief Takes a rotation quaternion and turns it into its Euler angle counterpart
 *
 * @param rotational_vector Rotation quaternion
 * @param degrees Quaternion degrees, not used
 *
 * @return Euler angle vector representation of the quaternion
 */
Vec3 quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}

/**
 * @brief Takes a gyroscope quaternion and turns it into its Euler 3D counterpart
 *
 * @param rotational_vector Gyroscope quaternion
 * @param degrees Quaternion degrees, not used
 *
 * @return Euler angle vector representation of the quaternion
 */
Vec3 quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}

std::tuple<float, float, float> euler_to_vector(float pitch, float yaw) {
    float x = cos(pitch) * cos(yaw);
    float y = cos(pitch) * sin(yaw);
    float z = sin(pitch);
    return {x, y, z};
}

float angular_difference(float pitch1, float yaw1, float pitch2, float yaw2) {
    auto [x1, y1, z1] = euler_to_vector(pitch1, yaw1);
    auto [x2, y2, z2] = euler_to_vector(pitch2, yaw2);

    float mag1 = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    float mag2 = sqrt(x2 * x2 + y2 * y2 + z2 * z2);

    float dot_product = x1 * x2 + y1 * y2 + z1 * z2;

    return std::acos(dot_product / (mag1 * mag2));
}

/**
 * @brief Generates a rotation matrix that transforms a vector by the rotations described in rpy_vec {roll, pitch, yaw} (in that order!)
 */
Eigen::Matrix3f generate_rotation_matrix(Vec3 rpy_vec) {
    float roll = rpy_vec.x;
    float pitch = rpy_vec.y;
    float yaw = rpy_vec.z;

    Eigen::Matrix3f Rx{
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    };

    Eigen::Matrix3f Ry{
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    };

    Eigen::Matrix3f Rz{
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    };

    return Rx * Ry * Rz;
}

/**
 * @brief Reads and returns the data from the sensor
 *
 * @return An orientation packet with orientation, acceleration, gyroscope, and magenetometer for all axes, along with temperature and pressure
 */
OrientationData HwImpl::read_orientation() {
    unsigned long currentTime = millis();
    deltaTime = (float) (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    OrientationData sensor_reading;
    sensor_reading.has_data = true;

    Vec3 euler = { 0, 0, 0 };
    switch (event.sensorId) {
        case SH2_ARVR_STABILIZED_RV: {
            euler = quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
            sensor_reading.reading_type = OrientationReadingType::FULL_READING;
            break;
        }
        case SH2_GYRO_INTEGRATED_RV: {
            sensor_reading.reading_type = OrientationReadingType::ANGULAR_VELOCITY_UPDATE;

            sensor_reading.angular_velocity.vx = event.un.gyroIntegratedRV.angVelX;
            sensor_reading.angular_velocity.vy = event.un.gyroIntegratedRV.angVelY;
            sensor_reading.angular_velocity.vz = event.un.gyroIntegratedRV.angVelZ;

            return sensor_reading;
        }
    }

    sensor_reading.euler.yaw = -euler.y;
    sensor_reading.euler.pitch = euler.x;
    sensor_reading.euler.roll = euler.z;

    sensor_reading.linear_acceleration.ax = -event.un.accelerometer.y;
    sensor_reading.linear_acceleration.ay = event.un.accelerometer.x;
    sensor_reading.linear_acceleration.az = event.un.accelerometer.z;

    Velocity velocity;
    velocity.vx = sensor_reading.linear_acceleration.ax * deltaTime + velocity.vx;
    velocity.vy = sensor_reading.linear_acceleration.ay * deltaTime + velocity.vy;
    velocity.vz = sensor_reading.linear_acceleration.az * deltaTime + velocity.vz;

    sensor_reading.orientation_velocity = velocity;

    sensor_reading.gx = -event.un.gyroscope.y;
    sensor_reading.gy = event.un.gyroscope.x;
    sensor_reading.gz = event.un.gyroscope.z;

    sensor_reading.magnetometer.mx = -event.un.magneticField.y;
    sensor_reading.magnetometer.my = event.un.magneticField.x;
    sensor_reading.magnetometer.mz = event.un.magneticField.z;

    sensor_reading.temperature = event.un.temperature.value;
    sensor_reading.pressure = event.un.pressure.value;

    Vec3 rotated_data{-euler.z, -euler.y, euler.x}; // roll, pitch, yaw

    // The guess & check method!
    // Quat --> euler --> rotation matrix --> reference&cur vector --> dot product for angle!

    Eigen::Matrix3f rot_matrix = generate_rotation_matrix(rotated_data);
    Eigen::Matrix<float, 1, 3> cur_ivec = {1, 0, 0};
    Eigen::Matrix<float, 1, 3> cur_vec = cur_ivec * rot_matrix;
    Eigen::Matrix<float, 1, 3> reference_vector = {0, 0, -1};

    float dot = cur_vec.dot(reference_vector);
    float cur_mag = cur_vec.norm();
    float ref_mag = reference_vector.norm();

    sensor_reading.tilt = 0;
    if (cur_mag != 0 && ref_mag != 0) {
        sensor_reading.tilt = acos(dot / (cur_mag * ref_mag));
    }

    return sensor_reading;
}

bool HwImpl::is_orientation_ready() {
    return imu.getSensorEvent(&event);
}

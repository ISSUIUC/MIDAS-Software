#include "sensors.h"
#include "Adafruit_BNO08x.h"

// global static instance of the sensor
Adafruit_BNO08x imu(BNO086_RESET);
#define REPORT_INTERVAL_US 5000

/**
 * @brief Initializes the bno sensor
 * 
 * @return Error Code
*/
ErrorCode OrientationSensor::init() {
    gpioPinMode(BNO086_RESET, OUTPUT);
    delay(100);
    // do whatever steps to initialize the sensor
    // if it errors, return the relevant error code
    if (!imu.begin_SPI(BNO086_CS, BNO086_INT)) {
        return ErrorCode::CannotConnectBNO;
    }
    Serial.println("Setting desired reports");
    if (!imu.enableReport(SH2_ARVR_STABILIZED_RV, REPORT_INTERVAL_US)) {
        return ErrorCode::CannotInitBNO;
    }
    return ErrorCode::NoError;
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
    euler.x = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));        // roll
    euler.y = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));       // yaw
    euler.z = -1 * atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));  // pitch
    return euler;
}

/**
 * @brief Takes a rotation quaternion and turns it into its Euler 3D counterpart
 * 
 * @param rotational_vector Rotation quaternion
 * @param degrees Quaternion degrees, not used
 * 
 * @return Euler 3D vector representation of the quaternion
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
 * @return Euler 3D vector representation of the quaternion
*/
Vec3 quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}

/**
 * @brief Reads and returns the data from the sensor
 * 
 * @return An orientation packet with orientation, acceleration, gyroscope, and magenetometer for all axes, along with temperature and pressure
*/
Orientation OrientationSensor::read() {
    // read from aforementioned global instance of sensor
    sh2_SensorValue_t event;
    Vec3 euler;
    if (imu.getSensorEvent(&event)) {
        switch (event.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                euler = quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
            case SH2_GYRO_INTEGRATED_RV:
                // faster (more noise?)
                euler = quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
                break;
        }

        Orientation sensor_reading;
        sensor_reading.has_data = true;

        sensor_reading.yaw = -euler.y;
        sensor_reading.pitch = euler.x;
        sensor_reading.roll = euler.z;

        sensor_reading.linear_acceleration.ax =  -event.un.accelerometer.y;
        sensor_reading.linear_acceleration.ay = event.un.accelerometer.x;
        sensor_reading.linear_acceleration.az = event.un.accelerometer.z;

        sensor_reading.gx = -event.un.gyroscope.y;
        sensor_reading.gy = event.un.gyroscope.x;
        sensor_reading.gz = event.un.gyroscope.z;

        sensor_reading.magnetometer.mx = -event.un.magneticField.y;
        sensor_reading.magnetometer.my = event.un.magneticField.x;
        sensor_reading.magnetometer.mz = event.un.magneticField.z;

        sensor_reading.temperature = event.un.temperature.value;
        sensor_reading.pressure = event.un.pressure.value;

        if (initial_flag == 0) {
            initial_orientation = sensor_reading;
            initial_flag = 1;
        }

        // calculate tilt from initial orientation
        Orientation deviation;
        deviation.yaw = min(abs(sensor_reading.yaw - initial_orientation.yaw), 2 * 3.14F - abs(sensor_reading.yaw - initial_orientation.yaw));
        deviation.pitch = min(abs(sensor_reading.pitch - initial_orientation.pitch), 2 * 3.14F - abs(sensor_reading.pitch - initial_orientation.pitch));

        sensor_reading.tilt = sqrt(pow(deviation.yaw, 2) + pow(deviation.pitch, 2));

        return sensor_reading;
    }
    return { .has_data = false };
}

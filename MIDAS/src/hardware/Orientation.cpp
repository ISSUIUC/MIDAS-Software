#include "sensors.h"

// #include sensor library
#include "Adafruit_BNO08x.h"
// #include <Eigen/Eigen>

// global static instance of the sensor
Adafruit_BNO08x imu(BNO086_RESET);
#define REPORT_INTERVAL_US 5000

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

Vec3 quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);

}

Vec3 quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}

Orientation OrientationSensor::read() {
    // read from aforementioned global instance of sensor
    sh2_SensorValue_t event;
    Vec3 euler;
    if (imu.getSensorEvent(&event)) {;
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
        // sensor_reading.yaw = euler.y;
        // sensor_reading.pitch = euler.z;
        // sensor_reading.roll = euler.x;

        // sensor_reading.linear_acceleration.ax = event.un.accelerometer.x;
        // sensor_reading.linear_acceleration.ay = event.un.accelerometer.y;
        // sensor_reading.linear_acceleration.az = event.un.accelerometer.z;

        // sensor_reading.gx = event.un.gyroscope.x;
        // sensor_reading.gy = event.un.gyroscope.y;
        // sensor_reading.gz = event.un.gyroscope.z;

        // sensor_reading.magnetometer.mx = event.un.magneticField.x;
        // sensor_reading.magnetometer.my = event.un.magneticField.y;
        // sensor_reading.magnetometer.mz = event.un.magneticField.z;

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
        // deviation.roll = sensor_reading.roll - initial_orientation.roll;

        sensor_reading.tilt = sqrt(pow(deviation.yaw, 2) + pow(deviation.pitch, 2));

        // Serial.println(sensor_reading.tilt);

        // Serial.print("yaw: ");
        // Serial.println(sensor_reading.yaw);
        // Serial.print(" pitch: ");
        // Serial.println(sensor_reading.pitch);
        // Serial.print(" roll: ");
        // Serial.println(sensor_reading.roll);

        return sensor_reading;
    }
    return { .has_data = false };
}

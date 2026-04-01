#pragma once

#include "sensor_data.h"

/**
 * @enum ReadingDiscriminant
 * 
 * @brief ID for each sensor, 0 is not used to make it easier to spot bugs
 */
enum ReadingDiscriminant {
    ID_IMU = 1, 
    ID_BAROMETER = 2,
    ID_VOLTAGE = 4,
    ID_GPS = 5,
    ID_MAGNETOMETER = 6,
    ID_KALMAN = 8,
    ID_FSM = 9,
    ID_PYRO = 10,
    ID_CAMERADATA = 11,
    ID_ANGULARKALMAN = 12,
    ID_SFLP = 13,
};


/**
 * @struct LoggerReading
 * 
 * @brief representation of data that will be logged
 * 
 * @note 
 * This struct isn't actually logged as-is, because if we did we'd waste extra space since
 * unions are the size of their largest member. This is just a reference struct.
 *
 * Instead, we use 4 bytes for the discriminant, 4 bytes for the timestamp, and then write the
 * actual data. No padding between inside these items or between readings.
 */
struct LoggedReading {
    ReadingDiscriminant discriminant;
    uint32_t timestamp_ms;
    union {
        IMU imu;
        IMU_SFLP sflp;
        Barometer barometer;
        Voltage voltage;
        GPS gps;
        Magnetometer magnetometer;
        KalmanData kalman;
        AngularKalmanData angular_kalman;
        FSMState fsm;
        PyroState pyro;
        CameraData cameradata;
    } data;
};
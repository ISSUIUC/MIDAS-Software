#pragma once

#include "sensor_data.h"
#define LOG_FMT_VERSION 1

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
    COUNT = 14, // Last element must be COUNT for HIL
};

constexpr uint8_t READING_DISC_COUNT = static_cast<uint8_t>(ReadingDiscriminant::COUNT);

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
        FSMData fsm;
        PyroState pyro;
        CameraData cameradata;
    } data;
};

/**
 * Associates a sensor type with its discriminant ID.
 * Used by data_logging.cpp for compile-time lookup, and parsed by log_enc.py
 * to build the discriminant-to-union-variant mapping for log metadata.
 *
 * Args: (type_name, discriminant_id, union_field_name)
 */
template<typename T>
constexpr ReadingDiscriminant get_discriminant();

#define ASSOCIATE(ty, id, field) template<> constexpr ReadingDiscriminant get_discriminant<ty>() { return ReadingDiscriminant::id; }

ASSOCIATE(IMU, ID_IMU, imu)
ASSOCIATE(IMU_SFLP, ID_SFLP, sflp)
ASSOCIATE(Barometer, ID_BAROMETER, barometer)
ASSOCIATE(Voltage, ID_VOLTAGE, voltage)
ASSOCIATE(GPS, ID_GPS, gps)
ASSOCIATE(Magnetometer, ID_MAGNETOMETER, magnetometer)
ASSOCIATE(KalmanData, ID_KALMAN, kalman)
ASSOCIATE(AngularKalmanData, ID_ANGULARKALMAN, angular_kalman)
ASSOCIATE(FSMData, ID_FSM, fsm)
ASSOCIATE(PyroState, ID_PYRO, pyro)
ASSOCIATE(CameraData, ID_CAMERADATA, cameradata)

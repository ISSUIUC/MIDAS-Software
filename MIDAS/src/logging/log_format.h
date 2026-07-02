#pragma once

#include "flight-systems/sensor_data.h"

#define LOG_FMT_VERSION 1

/**
 * @enum ReadingDiscriminant
 *
 * @brief Unique identifier assigned to each logged sensor or subsystem.
 *
 * These discriminants are written before every logged reading so the log parser
 * can determine which data structure follows in the binary log. The value `0`
 * is intentionally left unused to make invalid or corrupted records easier to
 * detect during parsing.
 *
 * @note
 * `COUNT` is not a valid discriminant. It represents the number of defined
 * discriminants and is primarily used for iteration and HIL simulation.
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
    COUNT = 14,
};

/**
 * @brief Total number of valid reading discriminants.
 *
 * This compile-time constant is primarily used by HIL simulation and other
 * code that needs to iterate through every supported log record type.
 */
constexpr uint8_t READING_DISC_COUNT =
    static_cast<uint8_t>(ReadingDiscriminant::COUNT);

/**
 * @struct LoggedReading
 *
 * @brief Logical representation of a single binary log entry.
 *
 * A logged reading consists of:
 *  - A sensor/subsystem identifier (`ReadingDiscriminant`)
 *  - A timestamp in milliseconds
 *  - The associated sensor data
 *
 * This structure is provided primarily as documentation of the log format and
 * for compile-time type information.
 *
 * @note
 * The logger does **not** write this structure directly to storage. Instead it
 * serializes each component individually:
 *   1. Discriminant
 *   2. Timestamp
 *   3. Raw sensor data
 *
 * Writing fields separately avoids the padding that would otherwise be present
 * due to the union, resulting in a compact binary log format.
 */
struct LoggedReading {
    /// Identifies the type of sensor data stored in this entry.
    ReadingDiscriminant discriminant;

    /// Timestamp of the reading in milliseconds.
    uint32_t timestamp_ms;

    /**
     * @brief Sensor data payload.
     *
     * Only one member is valid for any given log entry, as determined by
     * the corresponding value in `discriminant`.
     */
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
 * @brief Returns the log discriminant associated with a sensor data type.
 *
 * Template specializations provide a compile-time mapping between each sensor
 * structure and its corresponding `ReadingDiscriminant`. This allows the logger
 * to determine the correct record identifier without runtime lookups.
 *
 * The associations are also parsed by the metadata generation script
 * (`log_enc.py`) to build the binary log schema used by log analysis tools.
 *
 * @tparam T Sensor or subsystem data type.
 *
 * @return Compile-time `ReadingDiscriminant` corresponding to `T`.
 */
template<typename T>
constexpr ReadingDiscriminant get_discriminant();

/**
 * @brief Defines a compile-time association between a data type and its
 *        corresponding log discriminant.
 *
 * Expands into a template specialization of `get_discriminant<T>()`.
 *
 * @param ty Data type being associated.
 * @param id ReadingDiscriminant value.
 * @param field Corresponding union member name (used by metadata generation).
 */
#define ASSOCIATE(ty, id, field) \
template<> constexpr ReadingDiscriminant get_discriminant<ty>() { \
    return ReadingDiscriminant::id; \
}

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
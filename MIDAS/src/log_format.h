#pragma once

#include "sensor_data.h"

/**
 * @enum ReadingDiscriminant
 * 
 * @brief ID for each sensor, 0 is not used to make it easier to spot bugs
 */
enum ReadingDiscriminant {
    ID_LOWG = 1,
    ID_HIGHG = 2,
    ID_BAROMETER = 3,
    ID_CONTINUITY = 4,
    ID_VOLTAGE = 5,
    ID_GPS = 6,
    ID_MAGNETOMETER = 7,
    ID_ORIENTATION = 8,
    ID_LOWGLSM = 9,
    ID_FSM = 10,
    ID_KALMAN = 11,
    ID_PYRO = 12,
    ID_CAMERADATA = 13,
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
        LowGData low_g;
        HighGData high_g;
        Barometer barometer;
        Continuity continuity;
        Voltage voltage;
        GPS gps;
        Magnetometer magnetometer;
        Orientation orientation;
        LowGLSM lowg_lsm;
        KalmanData kalman;
        FSMState fsm;
        PyroState pyro;
        CameraData cameradata;
    } data;
};
#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "rocket_state.h"
#include "esp_eeprom.h"
#include "buzzer.h"

/**
 * @struct IMUSensor (HILSIM stub — calibration is a no-op).
 */
struct IMUSensor {
    enum IMUCalibrationState {
        NONE = 0,
        CALIB_PX = 1,
        CALIB_NX = 2,
        CALIB_PY = 3,
        CALIB_NY = 4,
        CALIB_PZ = 5,
        CALIB_NZ = 6,
        CALIB_DONE = 7
    };

    ErrorCode init();
    IMU read();
    IMU_SFLP read_sflp();

    void begin_calibration(BuzzerController&) {}
    void calib_reading(Acceleration, Acceleration, BuzzerController&, EEPROMController&) {}
    unsigned long get_time_since_calibration_start() { return 0; }
    void restore_calibration(EEPROMController&) {}
    void abort_calibration(BuzzerController&, EEPROMController&) {}

    IMUCalibrationState calibration_state = IMUCalibrationState::NONE;
    Acceleration calibration_sensor_bias = {0.0, 0.0, 0.0};
};

/**
 * @struct MagnetometerSensor (HILSIM stub — calibration is a no-op).
 */
struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();

    bool in_calibration_mode = false;
    void begin_calibration(BuzzerController&) {}
    void calib_reading(Magnetometer&, EEPROMController&, BuzzerController&) {}
    void restore_calibration(EEPROMController&) {}

    Magnetometer calibration_bias_hardiron = {0.0, 0.0, 0.0};
    Magnetometer calibration_bias_softiron = {1.0, 1.0, 1.0};
};

/**
 * @struct BarometerSensor (HILSIM stub).
 */
struct BarometerSensor {
    ErrorCode init();
    Barometer read();
};

/**
 * @struct VoltageSensor (HILSIM stub).
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

/**
 * @struct GPSSensor (HILSIM stub).
 */
struct GPSSensor {
    ErrorCode init();
    bool valid();
    GPS read();
    bool is_leap = false;
};

struct PyroTickData {
    const FSMData& fsm;
    const AngularKalmanData& akf;
    const KalmanData& ekf;
    const FSMConfiguration& fsm_configuration;
    CommandFlags& commands;
    double current_time;
    double time_since_launch;
};

/**
 * @struct Pyro (HILSIM stub).
 */
struct Pyro {
    ErrorCode init();
    PyroState tick(PyroTickData& data);
};

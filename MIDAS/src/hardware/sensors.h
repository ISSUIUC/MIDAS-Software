#pragma once

#include <util/errors.h>
#include "flight-systems/sensor_data.h"
#include "hardware/pins.h"
#include "TCAL9538.h"
#include "flight-systems/rocket_state.h"
#include "logging/esp_eeprom.h"
#include "util/buzzer.h"

/**
 * @struct IMUSensor
 *
 * @brief Interface for the onboard inertial measurement unit (IMU).
 *
 * Provides initialization, sensor data acquisition, and calibration
 * utilities for the LSM6DSV320X low-G/high-G accelerometers, gyroscope,
 * and Sensor Fusion Low Power (SFLP) engine.
 */
struct IMUSensor {

    /**
     * @brief States of the IMU calibration state machine.
     *
     * Calibration is performed by placing the rocket in each of the six
     * principal orientations (+X, -X, +Y, -Y, +Z, -Z).
     */
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

    /**
     * @brief Initializes the IMU.
     *
     * @return Error code indicating initialization status.
     */
    ErrorCode init();

    /**
     * @brief Reads the latest accelerometer and gyroscope data.
     *
     * @return IMU measurement packet.
     */
    IMU read();

    /**
     * @brief Reads the Sensor Fusion Low Power outputs.
     *
     * @return Sensor fusion measurement packet.
     */
    IMU_SFLP read_sflp();

    /**
     * @brief Begins the IMU calibration procedure.
     *
     * @param buzzer Buzzer used for user feedback.
     */
    void begin_calibration(BuzzerController& buzzer);

    /**
     * @brief Processes a calibration sample.
     *
     * @param lowg_reading Low-G accelerometer measurement.
     * @param highg_reading High-G accelerometer measurement.
     * @param buzzer_indicator Buzzer used for user feedback.
     * @param eeprom EEPROM controller used to store calibration data.
     */
    void calib_reading(Acceleration lowg_reading,
                       Acceleration highg_reading,
                       BuzzerController& buzzer_indicator,
                       EEPROMController& eeprom);

    /**
     * @brief Returns the elapsed calibration time.
     *
     * @return Time since calibration began in milliseconds.
     */
    unsigned long get_time_since_calibration_start() {
        return millis() - _calib_begin_timestamp;
    }

    /**
     * @brief Restores the saved calibration from EEPROM.
     *
     * @param eeprom EEPROM controller.
     */
    void restore_calibration(EEPROMController& eeprom);

    /**
     * @brief Aborts the current calibration procedure.
     *
     * @param buzzer Buzzer used for user feedback.
     * @param eeprom EEPROM controller.
     */
    void abort_calibration(BuzzerController& buzzer,
                           EEPROMController& eeprom);

    /// Current calibration state.
    IMUCalibrationState calibration_state = IMUCalibrationState::NONE;

    /// Computed high-G accelerometer bias.
    Acceleration calibration_sensor_bias = {0.0, 0.0, 0.0};

private:
    /// Number of accepted samples for the current orientation.
    int _calib_valid_readings = 0;

    /// Running sum of calibration offsets.
    float _calib_average = 0.0;

    /// Timestamp when calibration began.
    unsigned long _calib_begin_timestamp;

    /**
     * @brief Determines whether the current orientation is valid.
     */
    bool accept_calib_reading(float lowg_axis_reading,
                              float nominal_axis_value);

    /**
     * @brief Advances to the next calibration orientation.
     */
    void next_calib(BuzzerController& buzzer,
                    EEPROMController& eeprom);
};

/**
 * @struct MagnetometerSensor
 *
 * @brief Interface for the onboard MMC5983MA magnetometer.
 *
 * Provides magnetic field measurements and calibration routines for
 * hard-iron and soft-iron compensation.
 */
struct MagnetometerSensor {

    /**
     * @brief Initializes the magnetometer.
     *
     * @return Error code indicating initialization status.
     */
    ErrorCode init();

    /**
     * @brief Reads the latest magnetic field measurement.
     *
     * @return Magnetometer data packet.
     */
    Magnetometer read();

    /// Indicates whether calibration is currently active.
    bool in_calibration_mode = false;

    /**
     * @brief Begins magnetometer calibration.
     *
     * @param buzzer Buzzer used for user feedback.
     */
    void begin_calibration(BuzzerController& buzzer);

    /**
     * @brief Processes a magnetometer calibration sample.
     *
     * @param reading Current magnetometer measurement.
     * @param eeprom EEPROM controller.
     * @param buzzer Buzzer used for user feedback.
     */
    void calib_reading(Magnetometer& reading,
                       EEPROMController& eeprom,
                       BuzzerController& buzzer);

    /**
     * @brief Restores calibration values from EEPROM.
     *
     * @param eeprom EEPROM controller.
     */
    void restore_calibration(EEPROMController& eeprom);

    /// Hard-iron offset correction.
    Magnetometer calibration_bias_hardiron = {0.0, 0.0, 0.0};

    /// Soft-iron scale correction.
    Magnetometer calibration_bias_softiron = {1.0, 1.0, 1.0};

    /**
     * @brief Returns the elapsed calibration time.
     *
     * @return Time since calibration began in milliseconds.
     */
    unsigned long get_time_since_calibration_start() {
        return millis() - _calib_begin_timestamp;
    }

private:
    /// Computes and stores the completed calibration.
    void commit_calibration(EEPROMController& eeprom,
                            BuzzerController& buzzer);

    /// Performs sanity checks on the computed calibration.
    bool calibration_valid(const Magnetometer& b,
                           const Magnetometer& s);

    /// Maximum observed magnetic field values.
    Magnetometer _calib_max_axis;

    /// Minimum observed magnetic field values.
    Magnetometer _calib_min_axis;

    /// Calibration start timestamp.
    unsigned long _calib_begin_timestamp;

    /// Running sum of measured magnetic field magnitudes.
    double _calib_magnitude_sum = 0.0;

    /// Number of calibration samples collected.
    int _calib_num_datapoints = 0;

    /// Number of progress beeps already played.
    int _calib_beeping = 0;

    /**
     * @brief Duration of the magnetometer calibration procedure.
     */
    const unsigned long _calib_time = 60000;
};

/**
 * @struct BarometerSensor
 *
 * @brief Interface for the onboard barometer.
 */
struct BarometerSensor {

    /**
     * @brief Initializes the barometer.
     */
    ErrorCode init();

    /**
     * @brief Reads the latest barometer measurement.
     */
    Barometer read();
};

/**
 * @struct VoltageSensor
 *
 * @brief Interface for the onboard voltage monitor.
 */
struct VoltageSensor {

    /**
     * @brief Initializes the voltage monitor.
     */
    ErrorCode init();

    /**
     * @brief Reads the latest voltage measurement.
     */
    Voltage read();
};

/**
 * @struct GPSSensor
 *
 * @brief Interface for the onboard GNSS receiver.
 */
struct GPSSensor {

    /**
     * @brief Initializes the GPS receiver.
     */
    ErrorCode init();

    /**
     * @brief Returns whether a valid navigation solution is available.
     */
    bool valid();

    /**
     * @brief Reads the latest GPS navigation solution.
     */
    GPS read();

    /// Indicates whether the current year is a leap year.
    bool is_leap = false;
};

/**
 * @struct PyroTickData
 *
 * @brief Collection of inputs required for pyro state evaluation.
 */
struct PyroTickData {

    /// Current finite state machine state.
    const FSMData& fsm;

    /// Angular Kalman filter state estimate.
    const AngularKalmanData& akf;

    /// Translational Kalman filter state estimate.
    const KalmanData& ekf;

    /// Active FSM configuration.
    const FSMConfiguration& fsm_configuration;

    /// Mutable command flags.
    CommandFlags& commands;

    /// Current system time.
    double current_time;

    /// Time elapsed since launch.
    double time_since_launch;
};

/**
 * @struct Pyro
 *
 * @brief Controls all pyro firing logic.
 *
 * Handles manual pyro testing, autonomous deployment events, and
 * maintains the persistent state required by the pyro evaluator.
 */
struct Pyro {

    /**
     * @brief Initializes the pyro subsystem.
     */
    ErrorCode init();

    /**
     * @brief Computes the desired pyro output state.
     *
     * @param data Current flight state and sensor information.
     *
     * @return Desired pyro state.
     */
    PyroState tick(PyroTickData& data);

    /**
     * @brief Starts the manual pyro safety timer.
     */
    void set_pyro_safety();

    /**
     * @brief Clears the manual pyro safety timer.
     */
    void reset_pyro_safety();

private:
    /**
     * @brief Disarms every pyro channel.
     */
    void disarm_all_channels(PyroState& prev_state);

    /// Time at which the current manual firing began.
    double safety_pyro_start_firing_time;

    /// Indicates whether a manual firing has already occurred this cycle.
    bool safety_has_fired_pyros_this_cycle;

    /// Trigger timestamps for autonomous pyro events.
    double pyro_trigger_times[MIDAS_NUM_PYROS];

    /// Indicates whether delayed-event conditions have been rechecked.
    bool pyro_event_check[MIDAS_NUM_PYROS];

    /// Indicates whether each autonomous pyro event has completed.
    bool pyro_event_consumed[MIDAS_NUM_PYROS];
};
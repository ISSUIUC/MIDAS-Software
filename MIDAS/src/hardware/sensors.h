#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"
#include "TCAL9538.h"
#include "rocket_state.h"
#include "esp_eeprom.h"
#include "buzzer.h"


/**
 * @struct IMUSensor
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
    void begin_calibration(BuzzerController& buzzer);
    void calib_reading(Acceleration lowg_reading, Acceleration highg_reading, BuzzerController& buzzer_indicator, EEPROMController& eeprom);
    unsigned long get_time_since_calibration_start() { return millis() - _calib_begin_timestamp; }
    void restore_calibration(EEPROMController& eeprom);
    void abort_calibration(BuzzerController& buzzer, EEPROMController& eeprom);
    
    IMUCalibrationState calibration_state = IMUCalibrationState::NONE;
    Acceleration calibration_sensor_bias = {0.0, 0.0, 0.0};

    private:
    int _calib_valid_readings = 0;
    float _calib_average = 0.0;
    unsigned long _calib_begin_timestamp;

    bool accept_calib_reading(float lowg_axis_reading, float nominal_axis_value);
    void next_calib(BuzzerController& buzzer, EEPROMController& eeprom);


};

/**
 * @struct Magnetometer interface
 */
struct MagnetometerSensor {
    ErrorCode init();
    Magnetometer read();
};

/**
 * @struct Barometer interface
 */
struct BarometerSensor {
    ErrorCode init();
    Barometer read();
};

/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

/**
 * @struct GPS interface
 */
struct GPSSensor {
    ErrorCode init();
    bool valid();
    GPS read();
    bool is_leap = false;
};

/**
 * @struct Pyro interface
 */
struct Pyro {
    ErrorCode init();
    PyroState tick(FSMState fsm_state, AngularKalmanData angular_kalman_data, CommandFlags& telem_commands);

    void set_pyro_safety(); // Sets pyro_start_firing_time and has_fired_pyros.
    void reset_pyro_safety(); // Resets pyro_start_firing_time and has_fired_pyros. 
    
    private:
    void disarm_all_channels(PyroState& prev_state);
    void fire_pyro(int channel_idx, GpioAddress arm_pin, GpioAddress fire_pin);

    double safety_pyro_start_firing_time;    // Time when pyros have fired "this cycle" (pyro test) -- Used to only fire pyros for a time then transition to SAFE 
    bool safety_has_fired_pyros_this_cycle;  // If pyros have fired "this cycle" (pyro test) -- Allows only firing 1 pyro per cycle.
};

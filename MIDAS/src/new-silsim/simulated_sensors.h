# pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"
#include "TCAL9538.h"
#include "rocket_state.h"
#include "esp_eeprom.h"
#include "buzzer.h"
#include <fstream>


// timestamp_ms,sensor,imu.highg_acceleration.ax,imu.highg_acceleration.ay,
// imu.highg_acceleration.az,imu.lowg_acceleration.ax,imu.lowg_acceleration.ay,
// imu.lowg_acceleration.az,imu.angular_velocity.vx,imu.angular_velocity.vy,
// imu.angular_velocity.vz,barometer.temperature,barometer.pressure,barometer.altitude,
// voltage.continuity[0],voltage.continuity[1],voltage.continuity[2],voltage.continuity[3],
// voltage.v_Bat,voltage.v_Pyro,gps.latitude,gps.longitude,gps.altitude,gps.speed,gps.fix_type,
// gps.sats_in_view,gps.time,magnetometer.mx,magnetometer.my,magnetometer.mz,
// kalman.position.px,kalman.position.py,kalman.position.pz,kalman.velocity.vx,
// kalman.velocity.vy,kalman.velocity.vz,kalman.acceleration.ax,kalman.acceleration.ay,
// kalman.acceleration.az,fsm.state,fsm.current_motor,pyro.is_global_armed,
// pyro.channel_firing[0],pyro.channel_firing[1],pyro.channel_firing[2],pyro.channel_firing[3],
// pyro.pyro_event_consumed[0],pyro.pyro_event_consumed[1],pyro.pyro_event_consumed[2],
// pyro.pyro_event_consumed[3],cameradata.camera_state,cameradata.camera_voltage,
// angular_kalman.quaternion.w,angular_kalman.quaternion.x,angular_kalman.quaternion.y,
// angular_kalman.quaternion.z,angular_kalman.gyrobias[0],angular_kalman.gyrobias[1],
// angular_kalman.gyrobias[2],angular_kalman.sflp_tilt,angular_kalman.mq_tilt,
// angular_kalman.has_data,angular_kalman.yaw,angular_kalman.pitch,angular_kalman.roll,
// sflp.quaternion.w,sflp.quaternion.x,sflp.quaternion.y,sflp.quaternion.z,sflp.gravity.ax,
// sflp.gravity.ay,sflp.gravity.az,sflp.gyro_bias.vx,sflp.gyro_bias.vy,sflp.gyro_bias.vz

struct SILData {

    uint32_t timestamp;

    // imu
    float hg_ax, hg_ay, hg_az;
    float lg_ax, lg_ay, lg_az;
    float vx, vy, vz;


    // barometer:
    float temperature;
    float pressure;
    float altitude;

    // volatage:
    float pyro;
    float battery;

    // gps
    int32_t lat;
    int32_t lon;
    float alt;
    float speed;

    // magnetometer
    float mx;
    float my;
    float mz;

    //pyro

    //sflp
    float quat_w, quat_

};

class SimulatedSensor {
	private:
        std::ifstream csv_stream;
        virtual void update_data(int timestamp);
        virtual int read_data();


}

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

    // Calibration functions
    bool in_calibration_mode = false;
    void begin_calibration(BuzzerController& buzzer);
    void calib_reading(Magnetometer& reading, EEPROMController& eeprom, BuzzerController& buzzer);
    void restore_calibration(EEPROMController& eeprom);

    Magnetometer calibration_bias_hardiron = {0.0, 0.0, 0.0}; // hard iron offset -- "recenters" data on origin (0,0,0).
    Magnetometer calibration_bias_softiron = {1.0, 1.0, 1.0}; // soft iron offset -- scales per-axis data (Should be 3x3, but we'll try 1x3 for now.)

    unsigned long get_time_since_calibration_start() { return millis() - _calib_begin_timestamp; }

    private:
    void commit_calibration(EEPROMController& eeprom, BuzzerController& buzzer); // Calculate and commit the calibration to memory
    bool calibration_valid(const Magnetometer& b, const Magnetometer& s); // Calculate and sanity check calibration data
    /* Maximum value per-axis during calibration */
    Magnetometer _calib_max_axis; 
    /* Minimum value per-axis during calibration */
    Magnetometer _calib_min_axis;
    unsigned long _calib_begin_timestamp;
    double _calib_magnitude_sum = 0.0;
    int _calib_num_datapoints = 0;
    int _calib_beeping = 0;
    /* Magnetometer calibration isn't based on a per-axis calibration, but on getting as many datapoints as possible.
    For now let's try 60 sec */
    const unsigned long _calib_time = 60000;

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
 * @struct Pyro interface
 */
struct Pyro {
    ErrorCode init();
    PyroState tick(PyroTickData& data);

    void set_pyro_safety(); // Sets pyro_start_firing_time and has_fired_pyros.
    void reset_pyro_safety(); // Resets pyro_start_firing_time and has_fired_pyros. 
    
    private:
    void disarm_all_channels(PyroState& prev_state);
    
    double safety_pyro_start_firing_time;    // Time when pyros have fired "this cycle" (pyro test) -- Used to only fire pyros for a time then transition to SAFE 
    bool safety_has_fired_pyros_this_cycle;  // If pyros have fired "this cycle" (pyro test) -- Allows only firing 1 pyro per cycle.

    double pyro_trigger_times[MIDAS_NUM_PYROS]; // Storage for the time at which in-flight pyro event checks were triggered for each pyro.
    bool pyro_event_check[MIDAS_NUM_PYROS];     // Storage to indicate whether the pyro condition was checked (for pyro delay rule)
    bool pyro_event_consumed[MIDAS_NUM_PYROS];  // Storage for whether the pyro has attempted to have been fired.
};

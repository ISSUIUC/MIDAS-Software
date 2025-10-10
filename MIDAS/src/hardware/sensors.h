#pragma once

#include "errors.h"
#include "sensor_data.h"
#include "hardware/pins.h"
#include "TCAL9539.h"
#include "rocket_state.h"

/**
 * @struct LowG interface
 */
struct LowGSensor {
    ErrorCode init();
    LowGData read();
};

/**
 * @struct HighG interface
 */
struct HighGSensor {
    ErrorCode init();
    HighGData read();
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
 * @struct LowGLSM interface
 */
struct LowGLSMSensor {
    ErrorCode init();
    LowGLSM read();
};

/**
 * @struct Continuity interface
 */
struct ContinuitySensor {
    ErrorCode init();
    Continuity read();
};

/**
 * @struct Voltage interface
 */
struct VoltageSensor {
    ErrorCode init();
    Voltage read();
};

/**
 * @struct BNO interface
 */
struct OrientationSensor {
    Orientation initial_orientation;
    Quaternion initial_quaternion;
    uint8_t initial_flag;

    float prev_x = 0;
    float prev_y = 0;
    float prev_z = 0;
    float prev_tilt = 0;

    bool is_first_reset = true;

    ErrorCode init();
    Orientation read();
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
    PyroState tick(FSMState fsm_state, Orientation orientation, CommandFlags& telem_commands);

    void set_pyro_safety(); // Sets pyro_start_firing_time and has_fired_pyros.
    void reset_pyro_safety(); // Resets pyro_start_firing_time and has_fired_pyros.
    
    private:
    void disarm_all_channels(PyroState& prev_state);
    void fire_pyro(int channel_idx, GpioAddress arm_pin, GpioAddress fire_pin);

    double safety_pyro_start_firing_time;    // Time when pyros have fired "this cycle" (pyro test) -- Used to only fire pyros for a time then transition to SAFE 
    bool safety_has_fired_pyros_this_cycle;  // If pyros have fired "this cycle" (pyro test) -- Allows only firing 1 pyro per cycle.
};

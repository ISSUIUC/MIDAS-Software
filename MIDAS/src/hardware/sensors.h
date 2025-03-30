#pragma once

#include "hal.h"

struct HwImpl : HwInterface<HwImpl> {
    HwImpl() = default;

    ErrorCode init_all();

    LowGData read_low_g();
    LowGLSMData read_low_g_lsm();
    HighGData read_high_g();
    BarometerData read_barometer();
    ContinuityData read_continuity();
    VoltageData read_voltage();
    bool is_orientation_ready();
    OrientationData read_orientation();
    MagnetometerData read_magnetometer();
    bool is_gps_ready();
    GPSData read_gps();

    void set_global_arm(bool to_high);
    void set_pin_firing(int which, bool to_high);
};

//struct Pyro {
//    ErrorCode init();
//    PyroState tick(FSMState fsm_state, Orientation orientation, CommandFlags& telem_commands);
//
//    void set_pyro_safety(); // Sets pyro_start_firing_time and has_fired_pyros.
//    void reset_pyro_safety(); // Resets pyro_start_firing_time and has_fired_pyros.
//
//    private:
//    void disarm_all_channels(PyroState& prev_state);
//    void fire_pyro(int channel_idx, GpioAddress arm_pin, GpioAddress fire_pin);
//
//    double safety_pyro_start_firing_time;    // Time when pyros have fired "this cycle" (pyro test) -- Used to only fire pyros for a time then transition to SAFE
//    bool safety_has_fired_pyros_this_cycle;  // If pyros have fired "this cycle" (pyro test) -- Allows only firing 1 pyro per cycle.
//};

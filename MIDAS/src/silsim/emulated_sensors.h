#pragma once

#include "hal.h"

struct EmulatedHw : HwInterface<EmulatedHw> {
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

    void set_led(LED which, bool value);

    void transmit_bytes(uint8_t* memory, size_t count);
    bool receive_bytes(uint8_t* memory, size_t count, int wait_milliseconds);

    void set_global_arm(bool to_high);
    void set_pin_firing(Channel which, bool to_high);

    void set_camera_on(Camera which, bool on);
    void set_camera_source(Camera which);
    void set_video_transmit(bool on);
    uint8_t get_camera_state();

    LowGData lowGData;
    LowGLSMData lowGlsmData;
    HighGData highGData;
    BarometerData barometerData;
    ContinuityData continuityData;
    VoltageData voltageData;
    OrientationData orientationData;
    MagnetometerData magnetometerData;
    GPSData gpsData;

    std::vector<uint8_t> pending_transmissions;
};

#include "emulated_sensors.h"

ErrorCode EmulatedHw::init_all() {
    return ErrorCode::NoError;
}

LowGData EmulatedHw::read_low_g() {
    return lowGData;
}

LowGLSMData EmulatedHw::read_low_g_lsm() {
    return lowGlsmData;
}

HighGData EmulatedHw::read_high_g() {
    return highGData;
}

BarometerData EmulatedHw::read_barometer() {
    return barometerData;
}

ContinuityData EmulatedHw::read_continuity() {
    return continuityData;
}

VoltageData EmulatedHw::read_voltage() {
    return voltageData;
}

bool EmulatedHw::is_orientation_ready() {
    return true;
}

OrientationData EmulatedHw::read_orientation() {
    return orientationData;
}

MagnetometerData EmulatedHw::read_magnetometer() {
    return magnetometerData;
}

bool EmulatedHw::is_gps_ready() {
    return true;
}

GPSData EmulatedHw::read_gps() {
    return gpsData;
}

void EmulatedHw::set_led(LED which, bool value) {
    return;
}

void EmulatedHw::transmit_bytes(uint8_t* memory, size_t count) {
    return;
}

bool EmulatedHw::receive_bytes(uint8_t* memory, size_t count, int wait_milliseconds) {
    if (pending_transmissions.size() < count) {
        return false;
    } else {
        memcpy(memory, pending_transmissions.data(), count);
        pending_transmissions.erase(pending_transmissions.begin(), pending_transmissions.begin()+count);
        return true;
    }
}

void EmulatedHw::set_global_arm(bool to_high) {
    return;
}

void EmulatedHw::set_pin_firing(Channel which, bool to_high) {
    return;
}

void EmulatedHw::set_camera_on(Camera which, bool on) {
    return;
}

void EmulatedHw::set_camera_source(Camera which) {
    return;
}

void EmulatedHw::set_video_transmit(bool on) {
    return;
}

uint8_t EmulatedHw::get_camera_state() {
    return 0xFF;
}

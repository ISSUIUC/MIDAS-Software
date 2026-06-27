#pragma once

#include <array>
#include <cstdint>
#include "flight-systems/rocket_state.h"

#define MAX_TELEM_VOLTAGE_V 6.0f
#define MAX_TELEM_CONT_I 0.2f
#define MAX_KF_VPOSITION_M 100000.0f   // Max vertical position of KF data (m)
#define MAX_KF_LPOSITION_M 50000.0f   // Max lateral position of KF data (m)
#define MAX_ROLL_RATE_HZ 10.0f
#define MAX_ABS_ACCEL_RANGE_G 64
#define MAX_KF_XVELOCITY_MS 2000.0f

/**
 * @struct TelemetryPacket
 * 
 * @brief format of the telemetry packet
*/
struct TelemetryPacket {

    // GPS
    int32_t lat;
    int32_t lon;
    uint16_t alt; //15 bit meters, 1 bit command ack
    uint16_t baro_alt;

    /** @brief High-G accelerations mapped into fixed-point integer spanning (-64G, 64G]. **/
    uint16_t highg_ax; //16 bit accel (-64G, 64G]
    uint16_t highg_ay; //16 bit ay (-64G, 64G]
    uint16_t highg_az; //16 bit az (-64G, 64G]
    
    /** @brief Composite state: Upper 12 bits represent normalized tilt (0 to PI). Lower 4 bits hold the current FSMState enum. */
    uint16_t tilt_fsm; 
    /** @brief Main system battery voltage mapped using inv_convert_range up to MAX_TELEM_VOLTAGE_V. */
    uint8_t batt_volt;
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    

    uint8_t gpsfix_satcount; // lower 3 bits gps fix type, upper 5 bits total count of satellites in view
    uint8_t serial; // MIDAS Serial number
    /** @brief KF estimated horizontal X-axis velocity mapped onto MAX_KF_XVELOCITY_MS. */
    uint16_t kf_vx; // 16 bit meters/second
    /** @brief KF estimated horizontal Y-axis velocity mapped onto MAX_KF_XVELOCITY_MS. */
    uint16_t kf_vy; // 16 bit meters/second
    /** @brief KF estimated X-position mapped relative to MAX_KF_VPOSITION_M. */
    uint16_t kf_px; // 16 bit meters
    /** @brief KF estimated Y-position mapped relative to MAX_KF_LPOSITION_M. */
    uint16_t kf_py; // 16 bit meters
    /** @brief KF estimated Z-position mapped relative to MAX_KF_LPOSITION_M. */
    uint16_t kf_pz; // 16 bit meters

    uint32_t pyro; // 8 bit continuity x 4 channels (A, B, C, D), tracking pyrotechnic continuity in channels

    /** @brief Comprehensive bitwise system exception and error tracking collection flags. */
    MErrorFlags error_flags; 
    
    /** @brief Normalized absolute roll rate scaled relative to MAX_ROLL_RATE_HZ, spanning 0x00 to 0xFF. */
    uint8_t roll_rate;
    /** @brief Operating status byte originating from the on-board subsystem camera data. */
    uint8_t camera_state;
    /** @brief Derived representation of camera battery level computed via fractional scale relative to 9V. */
    uint8_t camera_batt_volt;
    
};



/** @brief Ground station command action flags parsed by the telemetry subsystem. */
enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_ARMED, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, CAM_ON, CAM_OFF, TOGGLE_CAM_VMUX, CALIB_ACCEL, CALIB_MAG };

/**
 * @struct TelemetryCommand
 * 
 * @brief format of the packet that telemetry receives
*/
struct TelemetryCommand {
    CommandType command;
    uint8_t serial;
    uint8_t serial_check;

    /**
     * @brief Performs a structural parity verification check on the received command envelope.
     * @details Compares the target hardware serial against the obfuscated bit-swapped check field.
     */

    bool valid() {
        return serial == (serial_check ^ 0xF2);
    }
};

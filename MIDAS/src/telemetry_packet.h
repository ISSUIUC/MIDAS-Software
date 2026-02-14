#pragma once

#include <array>
#include <cstdint>

#define MAX_TELEM_VOLTAGE_V 6.0f
#define MAX_TELEM_CONT_I 0.2f
#define MAX_KF_XPOSITION_M 20000.0f
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

    // High-G
    uint16_t highg_ax; //16 bit accel (-64G, 64G]
    uint16_t highg_ay; //16 bit ay (-64G, 64G]
    uint16_t highg_az; //16 bit az (-64G, 64G]
    
    uint16_t tilt_fsm; //12 bits tilt | 4 bits FSM
    uint8_t batt_volt;

    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;

    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    

    uint8_t callsign_gpsfix_satcount; //3 bits gpsfix, 4 bits sat count, 1 bit is_sustainer_callsign
    uint16_t kf_vx; // 16 bit meters/second
    uint16_t kf_px;  // 16 bit meters

    uint32_t pyro; // 7 bit continuity x 4 channels, 4 bit unused
    
    uint8_t roll_rate;
    uint8_t camera_state;
    uint8_t camera_batt_volt;
    
};


// Commands transmitted from ground station to rocket
enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_IDLE, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, CAM_ON, CAM_OFF, TOGGLE_CAM_VMUX };

/**
 * @struct TelemetryCommand
 * 
 * @brief format of the packet that telemetry receives
*/
struct TelemetryCommand {
    CommandType command;
    std::array<char, 3> verify;

    bool valid() {
        #ifdef IS_SUSTAINER
        return verify == std::array<char, 3>{{'B','R','K'}};
        #elif IS_BOOSTER
        return verify == std::array<char, 3>{{'A','R','K'}};
        #endif
    }
};
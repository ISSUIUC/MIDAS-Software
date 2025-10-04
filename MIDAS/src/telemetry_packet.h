#pragma once

#include <array>
#include <cstdint>

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
    uint16_t highg_ax;
    uint16_t highg_ay; //000|00000|0000
    uint16_t highg_az;
    
    uint16_t tilt_angle_battery_volts; //12 bit tilt angle, 4 bits will be [battery_volt]
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    
    uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
    uint16_t kf_vx; // 16 bit meters/second
    uint32_t pyro; // 7 bit continuity (4 bit tilt-> we made a new tilt angle 10 bit var, use that instead)
    
    uint8_t roll_rate;
    uint8_t camera_state;
    uint8_t kf_px;
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
        return verify == std::array<char, 3>{{'B','R','K'}};
    }
};

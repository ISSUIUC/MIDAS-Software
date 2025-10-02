#pragma once

#include <array>
#include <cstdint>

/**
 * @struct TelemetryPacket
 * 
 * @brief format of the telemetry packet
*/
#ifndef GNC_DATA
struct TelemetryPacket {
    int32_t lat;
    int32_t lon;
    uint16_t alt; //15 bit meters, 1 bit command ack
    uint16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_az; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint8_t batt_volt;
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    
    uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
    uint16_t kf_vx; // 16 bit meters/second
    uint32_t pyro; // 7 bit continuity 4 bit tilt
};
#endif


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

#ifdef GNC_DATA
struct TelemetryPacket {
    // Stores kalman data
    uint16_t k_pos_x;
    uint16_t k_pos_y;
    uint16_t k_pos_z;
    uint16_t k_vel_x;
    uint16_t k_vel_y;
    uint16_t k_vel_z;
    uint16_t k_acc_x;
    uint16_t k_acc_y;
    uint16_t k_acc_z;
    uint16_t k_altitude;

    // Raw sensor readings which are relevant
    uint16_t r_ax; 
    uint16_t r_ay; 
    uint16_t r_az;

    uint16_t r_pitch;
    uint16_t r_roll;
    uint16_t r_yaw;

    uint16_t r_tilt;

    // Other
    uint8_t fsm_callsign_ack; // 4 bit fsm state, 1 bit is_sustainer, 1 bit ack, 2 unused.
};
#endif
#pragma once
#include<stdint.h>

typedef uint32_t systime_t;

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


struct FullTelemetryData {
    systime_t timestamp;  //[0, 2^32]
    uint16_t altitude; // [0, 4096]
    float latitude; // [-90, 90]
    float longitude; // [-180, 180]
    float barometer_altitude; // [0, 4096]
    float highG_ax; // [-16, 16]
    float highG_ay; // [-16, 16]
    float highG_az; // [-16, 16]
    float battery_voltage; // [0, 5]
    uint8_t FSM_State; // [0, 255]
    float tilt_angle; // [-90, 90]
    float freq;
    float rssi;
    float sat_count;
    float pyros[4];
    bool is_sustainer;
    float kf_vx;
    bool kf_reset;
};

struct GNCTelemData {
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

struct DecodedGNCData {
    float k_pos_x;
    float k_pos_y;
    float k_pos_z;
    float k_vel_x;
    float k_vel_y;
    float k_vel_z;
    float k_acc_x;
    float k_acc_y;
    float k_acc_z;
    float k_altitude;

    // Raw sensor readings which are relevant
    float r_ax; 
    float r_ay; 
    float r_az;

    float r_pitch;
    float r_roll;
    float r_yaw;

    float r_tilt;

    // Other
    bool is_sustainer;
    uint8_t FSM_State; // [0, 255]
    bool kf_reset;    
    float freq;
    float rssi;
};

FullTelemetryData DecodePacket(const TelemetryPacket& packet, float frequency);
DecodedGNCData DecodePacketGNC(const GNCTelemData& packet, float frequency);

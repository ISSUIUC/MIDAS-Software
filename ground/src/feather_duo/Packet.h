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

FullTelemetryData DecodePacket(const TelemetryPacket& packet, float frequency);

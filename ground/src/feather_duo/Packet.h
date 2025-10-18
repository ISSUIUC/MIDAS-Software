#pragma once
#include<stdint.h>

typedef uint32_t systime_t;

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
    
    // If callsign bit (highest bit of fsm_callsign_satcount) is set, the callsign is KD9ZMJ
    //
    // If callsign bit (highest bit of fsm_callsign_satcount) is not set, the callsign is KD9ZPM
    

    uint8_t callsign_gpsfix_satcount; //3 bits gpsfix, 4 bits sat count, 1 bit is_sustainer_callsign
    uint16_t kf_vx; // 16 bit meters/second
    uint16_t kf_px;  // 16 bit meters
    uint32_t pyro; // 7 bit continuity x 4 channels, 4 unused bits
    
    uint8_t roll_rate;
    uint8_t camera_state;
    uint8_t camera_batt_volt;
    
};


struct FullTelemetryData {
    
    systime_t timestamp;  //[0, 2^32]
    uint16_t altitude;
    float latitude; // [-90, 90]
    float longitude; // [-180, 180]
    float barometer_altitude;
    
    float highG_ax; // [-64, 64]
    float highG_ay; // [-64, 64]
    float highG_az; // [-64, 64]
    
    float battery_voltage; // [0, MAX_TELEM_VOLTAGE_V]


    uint8_t FSM_State; // [0, 255]
    float tilt_angle; // [-90, 90]
    float freq;
    float rssi;
    
    uint8_t sat_count;
    uint8_t gps_fixtype;

    
    float pyros[4]; // 4-channel pyro continuity. // Pyro A number of continuous channels, Pyro B is measured current, Pyro C is expected current per channel, and Pyro D is pyro battery voltage 

    bool is_sustainer; // same as callsign

    float kf_vx; // kalman filter stuff
    float kf_px;

    bool cmd_ack; 

    float roll_rate_hz;
    uint8_t camera_state;
    float camera_batt_volt;
};

FullTelemetryData DecodePacket(const TelemetryPacket& packet, float frequency);

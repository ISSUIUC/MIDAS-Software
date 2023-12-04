#pragma once

#include <array>
#include <cstdint>

struct TelemetryDataLite {
    uint32_t timestamp;  //[0, 2^32]

    uint16_t barometer_pressure;  //[0, 4096]
    int16_t highG_ax;             //[128, -128]
    int16_t highG_ay;             //[128, -128]
    int16_t highG_az;             //[128, -128]
    int16_t bno_roll;             //[-4,4]
    int16_t bno_pitch;            //[-4,4]
    int16_t bno_yaw;              //[-4,4]

    float flap_extension;  //[0, 256]
};

struct TelemetryPacket {
    TelemetryDataLite datapoints[4];
    float gps_lat;
    float gps_long;
    float gps_alt;
    float yaw;
    float pitch;
    float roll;
    float gnc_state_x;
    float gnc_state_vx;
    float gnc_state_ax;
    float gnc_state_y;
    float gnc_state_vy;
    float gnc_state_ay;
    float gnc_state_z;
    float gnc_state_vz;
    float gnc_state_az;
    float gns_state_apo;
    int16_t mag_x;            //[-4, 4]
    int16_t mag_y;            //[-4, 4]
    int16_t mag_z;            //[-4, 4]
    int16_t gyro_x;           //[-4096, 4096]
    int16_t gyro_y;           //[-4096, 4096]
    int16_t gyro_z;           //[-4096, 4096]
    int16_t response_ID;      //[0, 2^16]
    int8_t rssi;              //[-128, 128]
    int8_t datapoint_count;   //[0,4]
    uint8_t voltage_battery;  //[0, 16]
    uint8_t FSM_State;        //[0,256]
    int16_t barometer_temp;   //[-128, 128]
    bool continuity_a;
    bool pyro_a;
    bool continuity_b;
    bool pyro_b;
    bool continuity_c;
    bool pyro_c;
    bool continuity_d;
    bool pyro_d;

    char callsign[8];
};

// Commands transmitted from ground station to rocket
enum CommandType {
    EMPTY = 0,
    SET_FREQ,
    SET_CALLSIGN
};

struct telemetry_command {
    CommandType command;
    int cmd_id;
    union {
        char callsign[8];
        float freq;
    };
    std::array<char, 6> verify;
};
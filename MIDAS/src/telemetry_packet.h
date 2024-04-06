#pragma once

#include <array>
#include <cstdint>

struct TelemetryDataLite {
    uint32_t timestamp;  //[0, 2^32]

    uint16_t barometer_pressure;  //[0, 4096]
    int16_t highG_ax;             //[128, -128]
    int16_t highG_ay;             //[128, -128]
    int16_t highG_az;             //[128, -128]
    int16_t lowg_ax;              // [-4, 4]
    int16_t lowg_ay;              // [-4, 4]
    int16_t lowg_az;              // [-4, 4]
    int16_t bno_roll;             //[-4,4]
    int16_t bno_pitch;            //[-4,4]
    int16_t bno_yaw;              //[-4,4]
};

struct TelemetryPacket {
    int8_t datapoint_count;   //[0,4]
    TelemetryDataLite datapoints[4];

    int32_t gps_lat;
    int32_t gps_long;
    float gps_alt;
    int16_t mag_x;            // [-4, 4]
    int16_t mag_y;            // [-4, 4]
    int16_t mag_z;            // [-4, 4]
    int16_t gyro_x;           // [-4096, 4096]
    int16_t gyro_y;           // [-4096, 4096]
    int16_t gyro_z;           // [-4096, 4096]
    int8_t rssi;              // [-128, 128]
    uint16_t voltage_battery;  // [0, 16]
    uint8_t FSM_state;        // [0,256]
    int16_t barometer_temp;   // [-128, 128]
    uint16_t sense_pyro;      // [0, 16]
    int8_t continuity[4];     // [-10, 10]
    /** Pyros bit format:
     *  Lowest order bit is 0
     *    0: pyros_armed[0]
     *    1: pyros_armed[1]
     *    2: pyros_armed[2]
     *    3: pyros_armed[3]
     *    4: pyros_firing[0]
     *    5: pyros_firing[1]
     *    6: pyros_firing[2]
     *    7: pyros_firing[3]
     */
    uint8_t pyros_bits;

    uint8_t telem_latency;    // [0, 1024]
    uint8_t log_latency;      // [0, 1024]

    bool is_booster;
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

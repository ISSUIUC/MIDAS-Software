#pragma once

#include <array>
#include <cstdint>

struct TelemetryPacket {
    uint16_t lat;
    uint16_t lon;
    uint16_t alt;
    int16_t highg_ax;
    int16_t highg_ay;
    int16_t highg_az;
    uint16_t baro_alt;
    uint16_t continuity;
    int8_t tilt_angle;
    uint8_t batt_volt;
    uint8_t pyro_volt;
    uint8_t fsm_satcount;
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

#pragma once

#include <array>
#include <cstdint>

struct TelemetryPacket {
    int32_t lat;
    int32_t lon;
    uint16_t alt;
    uint16_t baro_alt;
    uint16_t highg_ax; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_ay; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint16_t highg_az; //14 bit signed ax [-16,16) 2 bit tilt angle
    uint8_t batt_volt;
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

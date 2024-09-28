#pragma once
#include <array>

enum class CommandType { SET_FREQ, SET_CALLSIGN, ABORT, TEST_FLAP, EMPTY , RESET_KF };
// Commands transmitted from ground station to rocket

/**
 * @struct TelemetryCommand
 * 
 * @brief format of the packet that telemetry receives
*/
struct TelemetryCommand {
    CommandType command;
    int id;
    union {
        char callsign[8];
        float freq;
        bool do_abort;
    };
    std::array<char, 6> verify = {{'A', 'Y', 'B', 'E', 'R', 'K'}};
};
#pragma once
#include <array>

enum class CommandType: uint8_t { RESET_KF };
// Commands transmitted from ground station to rocket

/**
 * @struct TelemetryCommand
 * 
 * @brief format of the packet that telemetry receives
*/
struct TelemetryCommand {
    CommandType command;
    std::array<char, 3> verify = {{'B', 'R', 'K'}};

    bool valid() {
        return verify == std::array<char, 3>{{'B','R','K'}};
    }
};


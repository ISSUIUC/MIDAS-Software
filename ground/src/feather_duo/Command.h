#pragma once
#include<stdint.h>
#include<Array>

enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_IDLE, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, CAM_ON, CAM_OFF, TOGGLE_CAM_VMUX, EMPTY };
// Commands transmitted from ground station to rocket
struct TelemetryCommand {
    CommandType command;
    std::array<char, 3> verify = {{'B', 'R', 'K'}};
};
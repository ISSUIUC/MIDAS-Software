#pragma once
#include<stdint.h>
#include<array>

enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_IDLE, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, CAM_ON, CAM_OFF, TOGGLE_CAM_VMUX, CALIB_ACCEL, CALIB_MAG, EMPTY };
// Commands transmitted from ground station to rocket
struct TelemetryCommand {
    CommandType command;
    uint8_t serial;
    uint8_t serial_check;
};

TelemetryCommand buildTelemCmd(CommandType command, uint8_t serial){
    TelemetryCommand cmd;
    cmd.command = command;
    cmd.serial = serial;
    cmd.serial_check = serial ^ 0xF2;
    return cmd;
}
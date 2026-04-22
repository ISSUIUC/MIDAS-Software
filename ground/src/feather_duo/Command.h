#pragma once
#include<stdint.h>
#include<array>

enum class CommandType: uint8_t { RESET_KF, SWITCH_TO_SAFE, SWITCH_TO_PYRO_TEST, SWITCH_TO_ARMED, FIRE_PYRO_A, FIRE_PYRO_B, FIRE_PYRO_C, FIRE_PYRO_D, CAM_ON, CAM_OFF, TOGGLE_CAM_VMUX, CALIB_ACCEL, CALIB_MAG, EMPTY };
// Commands transmitted from ground station to rocket
struct TelemetryCommand {
    CommandType command;
    uint8_t serial;
    uint8_t serial_check;

    void setSerial(uint8_t sn){
        serial = sn;
        serial_check = sn ^ 0xF2;
    }
};
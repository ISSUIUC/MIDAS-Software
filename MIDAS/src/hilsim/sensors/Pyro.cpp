#include "sensors.h"
#include "../global_packet.h"

ErrorCode Pyro::init() {
    return ErrorCode::NoError;
}

PyroState Pyro::tick(FSMState fsm_state, Orientation orientation) {
    return PyroState();
    //tick
} // No new line for rhbog >:(
#include "../sensors.h"
#include "../kal_rocket.h"

ErrorCode Pyro::init() {
    return ErrorCode::NoError;
}

PyroState Pyro::tick(FSMState fsm_state, Orientation orientation, CommandFlags command_flags) {
    return PyroState();

    // TODO: Add actual pyro logic back..
}
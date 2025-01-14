#pragma once

#include <hardware_interface.h>

#include "sensor_data.h"
#include "finite-state-machines/fsm_states.h"

struct PyroController {
private:
    IPyroBackend& backend;

public:
    explicit PyroController(IPyroBackend& backend);

    ErrorCode init();
    PyroState tick_pyro(FSMState fsm_state, OrientationData orientation);
};

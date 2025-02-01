#pragma once

#include "hardware_interface.h"

/**
 * @class LEDController
 * 
 * @brief wraps functionality for LEDs
 */
class LEDController {
public:
    explicit LEDController(ILedBackend& backend);

    ErrorCode init();
    void update();
    void toggle(LED led);

private:
    ILedBackend& backend;

    int states[4] = {};
    bool targets[4] = {};
};

#pragma once

#include "errors.h"

class ILedBackend;

/**
 * @enum LED
 * 
 * @brief represents the different LEDS
*/
enum class LED {
    BLUE = 0,
    RED = 1,
    ORANGE = 2,
    GREEN = 3
};

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

#pragma once

#include "hal.h"
#include "errors.h"

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
    int states[4];
    int targets[4];

public:
    ErrorCode init();
    void update();

    void toggle(LED led);
};

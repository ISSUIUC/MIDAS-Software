#pragma once

#include "hal.h"
#include "errors.h"

enum class LED {
    BLUE = 0,
    RED = 1,
    ORANGE = 2,
    GREEN = 3
};

class LEDController {
    int states[4];
    int targets[4];

public:
    ErrorCode init();
    void update();

    void toggle(LED led);
};

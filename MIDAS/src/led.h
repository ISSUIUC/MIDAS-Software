#pragma once

#include "hal.h"

/**
 * @class LEDController
 * 
 * @brief wraps functionality for LEDs
 */
template<typename Hw>
class LEDController {
    bool states[4] = {};
    bool targets[4] = {};

    HwInterface<Hw>& hw;

public:
    LEDController(HwInterface<Hw>& hw) : hw(hw) { }

    void update() {
        for (int i = 0; i < 4; i++) {
            if (targets[i] != states[i]) {
                hw.set_led((LED) i, targets[i]);
                states[i] = targets[i];
            }
        }
    }

    void toggle(LED led) {
        int id = static_cast<int>(led);
        targets[id] = !targets[id];
    }

    void set(LED led, bool state) {
        targets[(int) led] = state;
    }
};

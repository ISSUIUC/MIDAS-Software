#include "led.h"
#include "hardware_interface.h"

LEDController::LEDController(ILedBackend& backend) : backend(backend) { }

/**
 * @brief Initializes LEDs
 * 
 * @return Error code
*/
ErrorCode LEDController::init() {
    return ErrorCode::NoError;
}

/**
 * @brief Toggles a specific LED's state
*/
void LEDController::toggle(LED led) {
    int id = static_cast<int>(led);
    targets[id] = !targets[id];
}

/**
 * @brief updates the LEDS to represent the current state the rocket is in
*/
void LEDController::update() {
    for (int i = 0; i < 4; i++) {
        if (targets[i] != states[i]) {
            backend.set(static_cast<LED>(i), targets[i]);
            states[i] = targets[i];
        }
    }
}

#include "led.h"
#include "TCAL9539.h"
#include "hardware/pins.h"

/**
 * @struct GpioAddress
 * 
 * @brief struct representing the LED pins
*/
static GpioAddress LED_pins[4] = {
    LED_BLUE,
    LED_RED,
    LED_ORANGE,
    LED_GREEN
};

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
    if (targets[id] == LOW) {
        targets[id] = HIGH;
    } else {
        targets[id] = LOW;
    }
}

/**
 * @brief updates the LEDS to represent the current state the rocket is in
*/
void LEDController::update() {
    for (int i = 0; i < 4; i++) {
        if (targets[i] != states[i]) {
            gpioDigitalWrite(LED_pins[i], targets[i]);
            states[i] = targets[i];
        }
    }
}
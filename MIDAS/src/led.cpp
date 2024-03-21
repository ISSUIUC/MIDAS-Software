#include "led.h"
#include "TCAL9539.h"
#include "hardware/pins.h"


static GpioAddress LED_pins[4] = {
    LED_BLUE,
    LED_RED,
    LED_ORANGE,
    LED_GREEN
};


ErrorCode LEDController::init() {
    return ErrorCode::NoError;
}

void LEDController::toggle(LED led) {
    int id = static_cast<int>(led);
    if (targets[id] == LOW) {
        targets[id] = HIGH;
    } else {
        targets[id] = LOW;
    }
}

void LEDController::update() {
    for (int i = 0; i < 4; i++) {
        if (targets[i] != states[i]) {
            Serial.print("Writing ");
            Serial.print(targets[i]);
            gpioDigitalWrite(LED_pins[i], targets[i]);
            states[i] = targets[i];
        }
    }
}

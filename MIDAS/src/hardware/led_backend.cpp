#include <TCAL9539.h>

#include "pins.h"
#include "led_backend.h"

static GpioAddress LED_pins[4] = {
    LED_BLUE,
    LED_RED,
    LED_ORANGE,
    LED_GREEN
};

void LedBackend::set(LED led, bool on) {
    gpioDigitalWrite(LED_pins[static_cast<int>(led)], on);
}

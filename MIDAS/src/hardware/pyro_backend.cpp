#include "pyro_backend.h"

#include <TCAL9539.h>

#include "pins.h"

bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
}

/**
 * @brief Initializes the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 *        have to do anything special and will always return NoError.
 */
ErrorCode PyroBackend::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // arm pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_ARM_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_ARM_PIN, OUTPUT));

    // fire pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_FIRE_PIN, OUTPUT));

    // todo revisit this lmao
    return ErrorCode::NoError;   // GPIO Driver always claimes it errored even when it doesn't.
}

void PyroBackend::arm_all() {
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, HIGH);
}

const GpioAddress ARM_PINS[4] = { PYROA_ARM_PIN, PYROB_ARM_PIN, PYROC_ARM_PIN, PYROD_ARM_PIN };
const GpioAddress FIRE_PINS[4] = { PYROA_FIRE_PIN, PYROB_FIRE_PIN, PYROC_FIRE_PIN, PYROD_FIRE_PIN };

void PyroBackend::fire_channel(int channel) {
    gpioDigitalWrite(ARM_PINS[channel], HIGH);
    gpioDigitalWrite(FIRE_PINS[channel], HIGH);
}

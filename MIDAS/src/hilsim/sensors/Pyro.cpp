#include "sensors.h"
#include "../kamaji/kal_rocket.h"

/**
 * @brief Returns true if the error_code signals failure.
 */
bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
}

/**
 * @brief Initializes the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 *        have to do anything special and will always return NoError.
 */
ErrorCode Pyro::init() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // fire pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_FIRE_PIN, OUTPUT));

    return ErrorCode::NoError; 
}

void Pyro::disarm_all_channels(PyroState& prev_state) {
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, LOW);
    gpioDigitalWrite(PYROA_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROB_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROC_FIRE_PIN, LOW);
    gpioDigitalWrite(PYROD_FIRE_PIN, LOW);

    prev_state.is_global_armed = false;
    
    for(size_t i = 0; i < 4; ++i) {
        // Update each channel's state sequentially
        prev_state.channel_firing[i] = false;
    }
}


PyroState Pyro::tick(FSMState fsm_state, Orientation orientation, CommandFlags& telem_commands) {
    disarm_all_channels(GLOBAL_DATA.pyro);
    return GLOBAL_DATA.pyro;
}

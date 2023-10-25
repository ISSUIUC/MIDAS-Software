#include "sensors.h"
#include "ADS7138-Q1.h"
#include "finite-state-machines/fsm_states.h"
#include "finite-state-machines/fsm.h"

GPIOADS7138 GPIO{};

/**
 * "Initializes" the pyro thread. The main initialization will be done by the GPIO expander, so the pyro thread doesn't
 * have to do anything special.
*/
ErrorCode PyroThread::init() {
    bool gpio_init = GPIO.init();
    if(gpio_init) {
        return ErrorCode::NoError;
    } else {
        return ErrorCode::PyroGPIOCouldNotBeInitialized;
    }
    
}

/**
 * Returns a new pyro struct, with data depending on whether or not each pyro channel should be firing.
*/
Pyro PyroThread::tick() {
    Pyro new_pyro = Pyro();
    // do some gnarly things
    return new_pyro;
}


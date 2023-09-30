#include "rocket.h"
#include "sensors.h"

#ifdef HILSIM
// HILSIM code
#endif

/**
 * Sets the config file and then starts all the threads using the config.
 */
void setup() {
    RocketConfig config { .sensors = Sensors(), .rocket_state = RocketState() };

    start_rocket(config);
}

void loop() {

}

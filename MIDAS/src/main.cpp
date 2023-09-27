#include "rocket.h"
#include "sensors.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */
void setup() {
    RocketConfig config { .sensors = Sensors(), .rocket_state = RocketState() };

    start_threads(config);
}

void loop() {

}

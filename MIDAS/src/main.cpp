#include "systems.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */
RocketSystems systems;

// According to this, don't ever do this
// https://docs.platformio.org/en/stable/advanced/unit-testing/structure/shared-code.html
// We shall then do this.
#ifndef PIO_UNIT_TESTING
void setup() {
    Serial.begin(9600);
    begin_systems(systems);
}

void loop() {

}
#endif // PIO_UNIT_TESTING

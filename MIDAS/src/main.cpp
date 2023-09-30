#include "rocket.h"
#include "sensors.h"

#ifdef HILSIM
// HILSIM code
#endif

/**
 * Sets the config file and then starts all the threads using the config.
 */
RocketConfig config;

void setup() {
    Serial.begin(9600);
    start_rocket(config);
}

void loop() {
}

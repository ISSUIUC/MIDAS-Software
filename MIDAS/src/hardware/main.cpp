#include "systems.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */
RocketSystems systems;

void setup() {
    Serial.begin(9600);
    begin_systems(&systems);
}

void loop() {

}

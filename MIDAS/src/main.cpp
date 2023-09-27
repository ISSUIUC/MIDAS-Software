#include "rocket.h"
#include "sensors.h"

void setup() {
    RocketConfig config { .sensors = Sensors(), .rocket_state = RocketState() };

    start_threads(config);
}

void loop() {

}

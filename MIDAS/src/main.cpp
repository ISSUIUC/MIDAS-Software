#include "rocket.h"
#include "sensors.h"

/**
 * Creates all threads for each sensor, FSM, Kalman algorithim, and data logging member
*/
void setup() {
    RocketConfig config { .sensors = Sensors(), .rocket_state = RocketState() };

    start_rocket(config);
}

void loop() {

}

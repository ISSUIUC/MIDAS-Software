#include "rocket.h"

#include "hal.h"


DECLARE_THREAD(data_logger, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(barometer, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(low_g, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(high_g, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(orientation, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(magnetometer, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(gps, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(gas, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(voltage, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(continuity, RocketConfig* arg) {
    while (true) {

    }
}

/**
 * Creates all threads for each sensor, FSM, Kalman algorithim, and data logging member
*/
void start_threads(RocketConfig config) {
    START_THREAD(data_logger, DATA_CORE, &config);
    START_THREAD(barometer, SENSOR_CORE, &config);
    START_THREAD(low_g, SENSOR_CORE, &config);
    START_THREAD(high_g, SENSOR_CORE, &config);
    START_THREAD(orientation, SENSOR_CORE, &config);
    START_THREAD(magnetometer, SENSOR_CORE, &config);
    START_THREAD(gps, SENSOR_CORE, &config);
    START_THREAD(gas, SENSOR_CORE, &config);
    START_THREAD(voltage, SENSOR_CORE, &config);
    START_THREAD(continuity, SENSOR_CORE, &config);

    vTaskStartScheduler();
}

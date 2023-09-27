#include "rocket.h"

#include "hal.h"
#include "sensors.h"

/**
 * These are all the functions that will run in each task
 * Each function has a `while(true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */
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
        LowGData reading = arg->sensors.low_g.read();
        arg->rocket_state.low_g.update(reading);

        THREAD_SLEEP(16);
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

DECLARE_THREAD(fsm, RocketConfig* arg) {
    while (true) {

    }
}

DECLARE_THREAD(kalman, RocketConfig* arg) {
    while (true) {

    }
}

/**
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
void start_threads(RocketConfig config) {
    START_THREAD(data_logger, DATA_CORE, &config);
    START_THREAD(barometer, SENSOR_CORE, &config);
    START_THREAD(low_g, SENSOR_CORE, &config);
    START_THREAD(high_g, SENSOR_CORE, &config);
    START_THREAD(orientation, SENSOR_CORE, &config);
    START_THREAD(magnetometer, SENSOR_CORE, &config);
    START_THREAD(gps, DATA_CORE, &config);
    START_THREAD(gas, SENSOR_CORE, &config);
    START_THREAD(voltage, SENSOR_CORE, &config);
    START_THREAD(continuity, SENSOR_CORE, &config);
    START_THREAD(fsm, SENSOR_CORE, &config);
    START_THREAD(kalman, SENSOR_CORE, &config);

    // since the START_THREAD macro uses local variables for the stacks of the threads, make sure
    //   those locals are still alive when we call vTaskStartScheduler()

    vTaskStartScheduler();
}

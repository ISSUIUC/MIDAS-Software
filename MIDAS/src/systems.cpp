#include "systems.h"

#include "hal.h"
#include "sensors.h"

#include "gnc/example_kf.h"

/**
 * These are all the functions that will run in each task
 * Each function has a `while(true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */
DECLARE_THREAD(data_logger, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("DATA");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("BARO");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(low_g, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("LOWG");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(gyroscope, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(10);
        // Serial.println("LOWG");
        arg->rocket_state.gyroscope.update(arg->sensors.gyroscope.read());
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(high_g, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("HIGHG");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("ORI");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("MAG");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(gps, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("GPS");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(gas, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("GAS");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("VOLT");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(continuity, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("conct");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(fsm, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("FSM");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(telem, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("TELEM");
    }
    vTaskDelete(NULL);
}

DECLARE_THREAD(kalman, RocketSystems* arg) {
    example_kf.initialize();
    while (true) {        
        THREAD_SLEEP(16);
        //Serial.println("KALMAN");
        example_kf.priori();
        example_kf.update();
    }
    vTaskDelete(NULL);
}

#define INIT_SENSOR(s) do { ErrorCode code = (s).init(); if (code != NoError) { return false; } } while (0)
bool init_sensors(Sensors& sensors) {
    // todo message on failure
    INIT_SENSOR(sensors.low_g);
    INIT_SENSOR(sensors.high_g);
    INIT_SENSOR(sensors.gyroscope);
    INIT_SENSOR(sensors.barometer);
    INIT_SENSOR(sensors.continuity);
    INIT_SENSOR(sensors.orientation);
    INIT_SENSOR(sensors.voltage);
    return true;
}
#undef INIT_SENSOR

/**
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
void begin_systems(RocketSystems& config) {
    bool success = init_sensors(config.sensors);
    if (!success) {
        // todo some message probably
        return;
    }
    START_THREAD(data_logger, DATA_CORE, &config);
    START_THREAD(barometer, SENSOR_CORE, &config);
    START_THREAD(low_g, SENSOR_CORE, &config);
    START_THREAD(gyroscope, SENSOR_CORE, &config);
    START_THREAD(high_g, SENSOR_CORE, &config);
    START_THREAD(orientation, SENSOR_CORE, &config);
    START_THREAD(magnetometer, SENSOR_CORE, &config);
    START_THREAD(gps, DATA_CORE, &config);
    START_THREAD(gas, SENSOR_CORE, &config);
    START_THREAD(voltage, SENSOR_CORE, &config);
    START_THREAD(continuity, SENSOR_CORE, &config);
    START_THREAD(fsm, SENSOR_CORE, &config);
    START_THREAD(kalman, SENSOR_CORE, &config);
    START_THREAD(telem, DATA_CORE, &config);

    while(true){
        THREAD_SLEEP(1000);
    }
}

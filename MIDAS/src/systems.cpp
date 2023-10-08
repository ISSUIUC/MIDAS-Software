#include "systems.h"

#include "hal.h"
#include "sensors.h"
#include "data_logging.h"

/**
 * These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 * 
 * @param name the name of the thread, replace with the actual name
 * @param arg the config file for the rocket
 */
DECLARE_THREAD(data_logger, RocketSystems* arg) {
    while (true) {
        log_data(arg->log_sink, arg->rocket_data);
        THREAD_SLEEP(50);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(barometer, RocketSystems* arg) {
    while (true) {
        Barometer reading = arg->sensors.barometer.read();
        arg->rocket_data.barometer.update(reading);

        THREAD_SLEEP(16);
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(low_g, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("LOWG");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(high_g, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("HIGHG");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("ORI");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("MAG");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(gps, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("GPS");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(gas, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("GAS");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("VOLT");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(continuity, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("conct");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(fsm, RocketSystems* arg) {
    while (true) {
        THREAD_SLEEP(16);
        //Serial.println("FSM");
    }
}

/**
 * See \ref data_logger_thread
 */
DECLARE_THREAD(kalman, RocketSystems* arg) {
    while (true) {        
        THREAD_SLEEP(16);
        //Serial.println("KALMAN");
    }
}

#define INIT_SENSOR(s) do { ErrorCode code = (s).init(); if (code != NoError) { return false; } } while (0)
bool init_sensors(Sensors& sensors, LogSink& log_sink) {
    // todo message on failure
    INIT_SENSOR(sensors.low_g);
    INIT_SENSOR(sensors.high_g);
    INIT_SENSOR(sensors.barometer);
    INIT_SENSOR(sensors.continuity);
    INIT_SENSOR(sensors.orientation);
    INIT_SENSOR(sensors.voltage);
    INIT_SENSOR(log_sink);
    return true;
}
#undef INIT_SENSOR

/**
 * Creates all threads for each sensor, FSM, Kalman algorithm, and data logging member
 * Starts thread scheduler to actually start doing jobs
*/
void begin_systems(RocketSystems& config) {
    bool success = init_sensors(config.sensors, config.log_sink);
    if (!success) {
        // todo some message probably
        return;
    }
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

    while (true) {
        THREAD_SLEEP(1000);
    }
}

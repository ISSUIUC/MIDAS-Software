#include "systems.h"

#include "hal.h"
#include "gnc/yessir.h"

#include <TCAL9539.h>

#if defined(IS_SUSTAINER) && defined(IS_BOOSTER)
#error "Only one of IS_SUSTAINER and IS_BOOSTER may be defined at the same time."
#elif !defined(IS_SUSTAINER) && !defined(IS_BOOSTER)
#error "At least one of IS_SUSTAINER and IS_BOOSTER must be defined."
#endif

#define ENABLE_TELEM

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */
DECLARE_THREAD(logger, RocketSystems* arg) {
    log_begin(arg->log_sink);
    while (true) {
        log_data(arg->log_sink, arg->rocket_data);

        arg->rocket_data.log_latency.tick();

        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(barometer, RocketSystems* arg) {
    // Reject single rogue barometer readings that are very different from the immediately prior reading
    // Will only reject a certain number of readings in a row
    Barometer prev_reading;
    constexpr float altChgThreshold = 200; // meters
    constexpr float presChgThreshold = 500; // milibars
    constexpr float tempChgThreshold = 10; // degrees C
    constexpr unsigned int maxConsecutiveRejects = 3;
    unsigned int rejects = maxConsecutiveRejects; // Always accept first reading
    while (true) {
        Barometer reading = arg->sensors.barometer.read();
        bool is_rogue = std::abs(prev_reading.altitude - reading.altitude) > altChgThreshold;
                        //std::abs(prev_reading.pressure - reading.pressure) > presChgThreshold ||
                        //std::abs(prev_reading.temperature - reading.temperature) > tempChgThreshold;
        // TODO: Log when we receive a rejection!
        if (is_rogue && rejects++ < maxConsecutiveRejects)
            arg->rocket_data.barometer.update(prev_reading); // Reuse old reading, reject new reading
        else {
            rejects = 0;
            arg->rocket_data.barometer.update(reading);
            prev_reading = reading; // Only update prev_reading with accepted readings
        }
        // Serial.print("Barometer ");
        // Serial.print(reading.altitude);
        // Serial.print(" ");
        // Serial.print(reading.pressure);
        // Serial.print(" ");
        // Serial.println(reading.temperature);
        THREAD_SLEEP(6);
    }
}

DECLARE_THREAD(accelerometers, RocketSystems* arg) {
    while (true) {
#ifdef IS_SUSTAINER
        LowGData lowg = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(lowg);
#endif
        LowGLSM lowglsm = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(highg);

        // Serial.print("Highg ");
        // Serial.print(highg.ax);
        // Serial.print(" ");
        // Serial.print(highg.ay);
        // Serial.print(" ");
        // Serial.println(highg.az);

        THREAD_SLEEP(2);
    }
}

DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        Orientation reading = arg->sensors.orientation.read();
        if (reading.has_data) {
            arg->rocket_data.orientation.update(reading);
        }
        // Serial.println("orient");
        THREAD_SLEEP(100);
    }
}

DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);  //data rate is 155hz so 7 is closest
        // Serial.print("mag ");
        // Serial.print(reading.mx);
        // Serial.print(" ");
        // Serial.print(reading.my);
        // Serial.print(" ");
        // Serial.println(reading.mz);
    }
}

// Ever device which communicates over i2c is on this thread to avoid interference
DECLARE_THREAD(i2c, RocketSystems* arg) {
    int i = 0;

    while (true) {
        if (i % 10 == 0) {
            if (arg->sensors.gps.valid()) {
                GPS reading = arg->sensors.gps.read();
                arg->rocket_data.gps.update(reading);
            } // Otherwise it's just dead :skull:

            FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
            CommandFlags& telem_commands = arg->rocket_data.command_flags;

            PyroState new_pyro_state = arg->sensors.pyro.tick(current_state, arg->rocket_data.orientation.getRecentUnsync(), telem_commands);
            arg->rocket_data.pyro.update(new_pyro_state);

            Continuity reading2 = arg->sensors.continuity.read();

            arg->rocket_data.continuity.update(reading2);

            Voltage reading3 = arg->sensors.voltage.read();
            arg->rocket_data.voltage.update(reading3);
            // Serial.println("i2c");
        }

        arg->led.update();
        i += 1;

        THREAD_SLEEP(10);
    }
}

// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm{};
    bool already_played_freebird = false;
    double last_time_led_flash = pdTICKS_TO_MS(xTaskGetTickCount());

    while (true) {
        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        StateEstimate state_estimate(arg->rocket_data);
        CommandFlags& telemetry_commands = arg->rocket_data.command_flags;
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        FSMState next_state = fsm.tick_fsm(current_state, state_estimate, telemetry_commands);

        arg->rocket_data.fsm_state.update(next_state);

        if (current_state == FSMState::STATE_SAFE) {
            if((current_time - last_time_led_flash) > 250) {
                // Flashes green LED at 4Hz while in SAFE mode.
                last_time_led_flash = current_time;
                arg->led.toggle(LED::GREEN);
            }
        } else {
            arg->led.set(LED::GREEN, LOW);
        }

        if ((current_state == FSMState::STATE_PYRO_TEST || current_state == FSMState::STATE_IDLE) && !arg->buzzer.is_playing()) {
            arg->buzzer.play_tune(warn_tone, WARN_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_LANDED && !arg->buzzer.is_playing()) {
            arg->buzzer.play_tune(land_tone, LAND_TONE_LENGTH);
        }

        if (current_state == FSMState::STATE_SUSTAINER_IGNITION && !already_played_freebird) {
            arg->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
            already_played_freebird = true;
        }
        // Serial.println("fsm");
        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

DECLARE_THREAD(kalman, RocketSystems* arg) {
    // Orientation initial_orientation = arg->rocket_data.orientation.getRecent();
    // Barometer initial_barom_buf = arg->rocket_data.barometer.getRecent();
    // LowGData initial_accelerometer = arg->rocket_data.low_g.getRecent();
    //yessir.initialize(initial_orientation, initial_barom_buf, initial_accelerations);
    yessir.initialize(arg);
    TickType_t last = xTaskGetTickCount();
    
    while (true) {
        if(arg->rocket_data.command_flags.should_reset_kf){
            yessir.initialize(arg);
            TickType_t last = xTaskGetTickCount();
            arg->rocket_data.command_flags.should_reset_kf = false;
        }
        // add the tick update function
        Barometer current_barom_buf = arg->rocket_data.barometer.getRecent();
        Orientation current_orientation = arg->rocket_data.orientation.getRecent();
        HighGData current_accelerometer = arg->rocket_data.high_g.getRecent();
        FSMState FSM_state = arg->rocket_data.fsm_state.getRecent();
        Acceleration current_accelerations = {
            .ax = current_accelerometer.ax,
            .ay = current_accelerometer.ay,
            .az = current_accelerometer.az
        };
        float dt = pdTICKS_TO_MS(xTaskGetTickCount() - last) / 1000.0f;

        yessir.tick(dt, 13.0, current_barom_buf, current_accelerations, current_orientation, FSM_state);
        KalmanData current_state = yessir.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();
        // Serial.println("Kalman");
        THREAD_SLEEP(50);
    }
}

void handle_tlm_command(TelemetryCommand& command, RocketSystems* arg, FSMState current_state) {
// maybe we should move this somewhere else but it can stay here for now
Serial.println((int) command.command);
    switch(command.command) {
        case CommandType::RESET_KF:
            arg->rocket_data.command_flags.should_reset_kf = true;
            break;
        case CommandType::SWITCH_TO_SAFE:
            arg->rocket_data.command_flags.should_transition_safe = true;
            break;
        case CommandType::SWITCH_TO_PYRO_TEST:
            arg->rocket_data.command_flags.should_transition_pyro_test = true;
            Serial.println("Changing to pyro test");
            break;
        case CommandType::SWITCH_TO_IDLE:
            arg->rocket_data.command_flags.should_transition_idle = true;
            break;
        case CommandType::FIRE_PYRO_A:
            if (current_state == FSMState::STATE_PYRO_TEST) {
                arg->rocket_data.command_flags.should_fire_pyro_a = true;
            }
            break;
        case CommandType::FIRE_PYRO_B:
            if (current_state == FSMState::STATE_PYRO_TEST) {
                arg->rocket_data.command_flags.should_fire_pyro_b = true;
            }
            break;
        case CommandType::FIRE_PYRO_C:
            if (current_state == FSMState::STATE_PYRO_TEST) {
                arg->rocket_data.command_flags.should_fire_pyro_c = true;
            }
            break;
        case CommandType::FIRE_PYRO_D:
            if (current_state == FSMState::STATE_PYRO_TEST) {
                arg->rocket_data.command_flags.should_fire_pyro_d = true;
            }
            break;
        default:
            break; // how
    }
}
DECLARE_THREAD(telemetry, RocketSystems* arg) {
    double launch_time = 0;

    while (true) {
        arg->tlm.transmit(arg->rocket_data, arg->led);

        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();

        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        if (current_state == FSMState::STATE_IDLE) {
            launch_time = current_time;
        }

        if (current_state == FSMState(STATE_IDLE) || current_state == FSMState(STATE_SAFE) || current_state == FSMState(STATE_PYRO_TEST) || (current_time - launch_time) > 1800000) {
            TelemetryCommand command;
            if (arg->tlm.receive(&command, 2000) == 0) {
                Serial.print(command.verify[0]);
                Serial.print(command.verify[1]);
                Serial.print(command.verify[2]);
                Serial.println("rec command");
                if (command.valid()) {
                    Serial.println("Valid command");
                    arg->tlm.acknowledgeReceived();
                    handle_tlm_command(command, arg, current_state);
                }
            }
        }
        THREAD_SLEEP(1);
        // Serial.println("Telemetry");
    }
}

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode init_systems(RocketSystems& systems) {
    gpioDigitalWrite(LED_ORANGE, HIGH);
#ifdef IS_SUSTAINER
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.orientation);
#endif
    INIT_SYSTEM(systems.log_sink);
    INIT_SYSTEM(systems.sensors.high_g);
    INIT_SYSTEM(systems.sensors.low_g_lsm);
    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.continuity);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.pyro);
    INIT_SYSTEM(systems.led);
    INIT_SYSTEM(systems.buzzer);
    #ifdef ENABLE_TELEM
        INIT_SYSTEM(systems.tlm);
    #endif
    INIT_SYSTEM(systems.sensors.gps);
    gpioDigitalWrite(LED_ORANGE, LOW);
    return NoError;
}
#undef INIT_SYSTEM


/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
[[noreturn]] void begin_systems(RocketSystems* config) {
    Serial.println("Starting Systems...");
    ErrorCode init_error_code = init_systems(*config);
    if (init_error_code != NoError) {
        // todo some message probably
        Serial.print("Had Error: ");
        Serial.print((int) init_error_code);
        Serial.print("\n");
        Serial.flush();
        update_error_LED(init_error_code);
        while (true) {

        }
    }

#ifdef IS_SUSTAINER
    START_THREAD(orientation, SENSOR_CORE, config, 10);
#endif

    START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(accelerometers, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(i2c, SENSOR_CORE, config, 9);
    START_THREAD(magnetometer, SENSOR_CORE, config, 11);
    START_THREAD(kalman, SENSOR_CORE, config, 7);
    START_THREAD(fsm, SENSOR_CORE, config, 8);
    START_THREAD(buzzer, SENSOR_CORE, config, 6);
    #ifdef ENABLE_TELEM
    START_THREAD(telemetry, SENSOR_CORE, config, 15);
    #endif

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while (true) {
        THREAD_SLEEP(1000);
        Serial.print("Running (Log Latency: ");
        Serial.print(config->rocket_data.log_latency.getLatency());
        Serial.println(")");
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}

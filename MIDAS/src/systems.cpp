#include "systems.h"

#include "hal.h"
#include "gnc/ekf.h"

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
        LowGData lowg = arg->sensors.low_g.read();
        arg->rocket_data.low_g.update(lowg);
        LowGLSM lowglsm = arg->sensors.low_g_lsm.read();
        arg->rocket_data.low_g_lsm.update(lowglsm);
        HighGData highg = arg->sensors.high_g.read();
        arg->rocket_data.high_g.update(highg);

        THREAD_SLEEP(2);
    }
}

DECLARE_THREAD(orientation, RocketSystems* arg) {
    while (true) {
        Orientation orientation_holder = arg->rocket_data.orientation.getRecent();
        Orientation reading = arg->sensors.orientation.read();
        if (reading.has_data) {
            if(reading.reading_type == OrientationReadingType::ANGULAR_VELOCITY_UPDATE) {
                orientation_holder.angular_velocity.vx = reading.angular_velocity.vx;
                orientation_holder.angular_velocity.vy = reading.angular_velocity.vy;
                orientation_holder.angular_velocity.vz = reading.angular_velocity.vz;
            } else {
                float old_vx = orientation_holder.angular_velocity.vx;
                float old_vy = orientation_holder.angular_velocity.vy;
                float old_vz = orientation_holder.angular_velocity.vz;
                orientation_holder = reading;
                orientation_holder.angular_velocity.vx = old_vx;
                orientation_holder.angular_velocity.vy = old_vy;
                orientation_holder.angular_velocity.vz = old_vz;
            }

            arg->rocket_data.orientation.update(orientation_holder);
        }

        THREAD_SLEEP(100);
    }
}

DECLARE_THREAD(magnetometer, RocketSystems* arg) {
    while (true) {
        Magnetometer reading = arg->sensors.magnetometer.read();
        arg->rocket_data.magnetometer.update(reading);
        THREAD_SLEEP(50);
    }
}

DECLARE_THREAD(gps, RocketSystems* arg) {
    while(true) {
        if(arg->sensors.gps.valid()) {
            GPS reading = arg->sensors.gps.read();
            arg->rocket_data.gps.update(reading);
        }
        //GPS waits internally
        THREAD_SLEEP(1);
    }
}

DECLARE_THREAD(pyro, RocketSystems* arg) {
    while(true) {
        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        CommandFlags& command_flags = arg->rocket_data.command_flags;

        PyroState new_pyro_state = arg->sensors.pyro.tick(current_state, arg->rocket_data.orientation.getRecentUnsync(), command_flags);
        arg->rocket_data.pyro.update(new_pyro_state);

        arg->led.update();

        THREAD_SLEEP(10);
    }   
}

DECLARE_THREAD(voltage, RocketSystems* arg) {
    while (true) {
        Continuity reading = arg->sensors.continuity.read();
        Voltage reading2 = arg->sensors.voltage.read();;

        arg->rocket_data.continuity.update(reading);
        arg->rocket_data.voltage.update(reading2);

        THREAD_SLEEP(100);
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
        
        // FSM-based camera control
        if(arg->rocket_data.command_flags.FSM_should_set_cam_feed_cam1) { 
            // Swap camera feed to MUX 1 (Side-facing camera) at launch.
            arg->rocket_data.command_flags.FSM_should_set_cam_feed_cam1 = false;
            arg->b2b.camera.vmux_set(SIDE_CAMERA);
        }

        if(arg->rocket_data.command_flags.FSM_should_swap_camera_feed) { 
            // Swap camera feed to MUX 2 (recovery bay camera)
            arg->rocket_data.command_flags.FSM_should_swap_camera_feed = false;
            arg->b2b.camera.vmux_set(BULKHEAD_CAMERA);
        }

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
    ekf.initialize(arg);
    // Serial.println("Initialized ekf :(");
    TickType_t last = xTaskGetTickCount();
    
    while (true) {
        if(arg->rocket_data.command_flags.should_reset_kf){
            ekf.initialize(arg);
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
        float timestamp = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000.0f;
        ekf.tick(dt, 13.0, current_barom_buf, current_accelerations, current_orientation, FSM_state);
        KalmanData current_state = ekf.getState();

        arg->rocket_data.kalman.update(current_state);

        last = xTaskGetTickCount();
        // Serial.println("Kalman");
        THREAD_SLEEP(50);
    }
}


void handle_tlm_command(TelemetryCommand& command, RocketSystems* arg, FSMState current_state) {
    // maybe we should move this somewhere else but it can stay here for now
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
        case CommandType::CAM_ON:
            arg->b2b.camera.camera_on(CAM_1);
            arg->b2b.camera.camera_on(CAM_2);
            arg->b2b.camera.vtx_on();
            break;
        case CommandType::CAM_OFF:
            arg->b2b.camera.camera_off(CAM_1);
            arg->b2b.camera.camera_off(CAM_2);
            arg->b2b.camera.vtx_off();
            break;
        case CommandType::TOGGLE_CAM_VMUX:
            arg->b2b.camera.vmux_toggle();
            break;
        default:
            break; // how
    }
}

DECLARE_THREAD(telemetry, RocketSystems* arg) {
    double launch_time = 0;
    bool has_triggered_vmux_fallback = false;

    while (true) {

        arg->tlm.transmit(arg->rocket_data, arg->led);

        FSMState current_state = arg->rocket_data.fsm_state.getRecentUnsync();
        double current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        if (current_state == FSMState::STATE_IDLE) {
            launch_time = current_time;
            has_triggered_vmux_fallback = false;
        }

        if ((current_time - launch_time) > 80000 && !has_triggered_vmux_fallback) {
            // THIS IS A HARDCODED VALUE FOR AETHER 3/15/2025
            // If the rocket has been in flight for over 80 seconds, we swap the FSM camera feed to the bulkhead camera
            has_triggered_vmux_fallback = true;
            arg->rocket_data.command_flags.FSM_should_swap_camera_feed = true;
        }

        if (current_state == FSMState(STATE_IDLE) || current_state == FSMState(STATE_SAFE) || current_state == FSMState(STATE_PYRO_TEST) || (current_time - launch_time) > 1800000) {
            TelemetryCommand command;
            if (arg->tlm.receive(&command, 200)) {
                if (command.valid()) {
                    arg->tlm.acknowledgeReceived();
                    handle_tlm_command(command, arg, current_state);
                }
            }
        }
        THREAD_SLEEP(1);
    }
}

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode init_systems(RocketSystems& systems) {
    gpioDigitalWrite(LED_ORANGE, HIGH);
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.orientation);
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
    INIT_SYSTEM(systems.b2b);
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

    START_THREAD(orientation, SENSOR_CORE, config, 10);
    START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(accelerometers, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(gps, SENSOR_CORE, config, 8);
    START_THREAD(voltage, SENSOR_CORE, config, 9);
    START_THREAD(pyro, SENSOR_CORE, config, 14);
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

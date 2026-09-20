// KAL - Kamaji Abstraction Layer
#include "kal.h"
#include "shell/midas_shell.h"

// ---- GLOBALS ----
SDSink sink;
RocketSystems systems{.log_sink = sink};

// ---- INIT SENSORS ----
void k_init_sensordata() {
    KRocketData* arg = &GLOBAL_DATA;

    // This list should run parallel to the list defined in data_logging. This macro will perform the opposite task,
    // mapping disc IDs to their locations in the struct.
    ASSOCIATE(arg->imu, ID_IMU);
    ASSOCIATE(arg->sflp, ID_SFLP);
    ASSOCIATE(arg->barometer, ID_BAROMETER);
    ASSOCIATE(arg->voltage, ID_VOLTAGE);
    ASSOCIATE(arg->gps, ID_GPS);
    ASSOCIATE(arg->magnetometer, ID_MAGNETOMETER);
    ASSOCIATE(arg->kalman, ID_KALMAN);
    ASSOCIATE(arg->angular_kalman_data, ID_ANGULARKALMAN);
    ASSOCIATE(arg->fsm_state, ID_FSM);
    ASSOCIATE(arg->pyro, ID_PYRO);
    ASSOCIATE(arg->cam_data, ID_CAMERADATA);
}

// ---- Kamaji Thread ----
DECLARE_THREAD(hilsim, RocketSystems* arg) {
    uint8_t header[5];
    uint8_t crc_buf[2];
    uint32_t timestamp;
    uint8_t discriminant;
    size_t data_size;
    uint8_t data_buf[128];
    uint16_t crc;

    sys_flags_t sflags;
    
    while (true) {
        int delim;
        do {
            k_tick();
            if (Serial.available() == 0) {
                THREAD_SLEEP(1);
            }
            delim = Serial.read();

            if (sflags.fsm_target != FSMState::FSM_STATE_COUNT) {
                arg->rocket_data.fsm_state.update({sflags.fsm_target, sflags.fsm_target_motor});
                sflags.fsm_target = FSMState::FSM_STATE_COUNT;
            }

        } while(delim != '$' && delim != '#' && delim != '&');

        if(delim == '&') {
            START_THREAD(shell, DATA_CORE, arg, 5);
            vTaskDelete(NULL);
            continue;
        }

        if(delim == '#') {
          if (!k_read_exact(data_buf, 16, 10)) continue;

          // Sys message is 14 byte data, 2 byte crc  
          memcpy(&crc, data_buf + 14, sizeof(crc_buf));
          k_handle_sys_msg(data_buf, crc, &sflags);
          continue;
        }
            
        if(delim == '$') {
          if(!k_read_exact(header, sizeof(header), 10)) continue;

          memcpy(&timestamp, &header[0], 4);
          discriminant = header[4];
          data_size = k_get_discriminant_size(discriminant);

          if (!k_read_exact(data_buf, (size_t)data_size, 10)) continue;
          if (!k_read_exact(crc_buf, sizeof(crc_buf), 5)) continue;

          memcpy(&crc, crc_buf, sizeof(crc_buf));
          k_handle_reading(timestamp, discriminant, data_buf, data_size, crc);
          
        }

        k_tick();
        THREAD_SLEEP(1);
    }
}

// Run the Kamaji process
[[noreturn]] void k_start() {
    // These mutexes normally get created in begin_systems() but HILSIM bypasses that.
    static StaticSemaphore_t kal_spi_mutex_buffer;
    spi_mutex = xSemaphoreCreateMutexStatic(&kal_spi_mutex_buffer);
    static StaticSemaphore_t kal_i2c_mutex_buffer;
    i2c_mutex = xSemaphoreCreateMutexStatic(&kal_i2c_mutex_buffer);

    // logger thread is not started in HILSIM, but FSM transitions write to meta_logging.summary.
    // Point it at a static dummy so those writes don't fault.
    static MetalogSummary kal_meta_summary;
    systems.meta_logging.summary = &kal_meta_summary;

    // real midas setup
    INIT_SYSTEM(systems.eeprom);

    // To init log sink fast, we'll set its filenumber before initing it.
    systems.log_sink.current_file_no = systems.eeprom.data.sd_file_num_last;

    INIT_SYSTEM(systems.log_sink);

    // Then, after initing we will read back the filenumber to EEPROM.
    systems.eeprom.data.sd_file_num_last = systems.log_sink.current_file_no;

    INIT_SYSTEM(systems.sensors.barometer);
    INIT_SYSTEM(systems.sensors.magnetometer);
    INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.sensors.pyro);
    INIT_SYSTEM(systems.led);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.b2b);
    INIT_SYSTEM(systems.tlm);

    m_shell_setup();
    systems.shell = &m_shell_inst;
    m_shell_init_commands(systems.shell);

    Serial.println("Starting Systems...");
    RocketSystems* config = &systems;

    START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(imuthread, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(gps, SENSOR_CORE, config, 8);
    START_THREAD(voltage, SENSOR_CORE, config, 9);
    START_THREAD(pyro, SENSOR_CORE, config, 14);
    START_THREAD(magnetometer, SENSOR_CORE, config, 11);
    START_THREAD(cam, SENSOR_CORE, config, 16);
    START_THREAD(kalman, SENSOR_CORE, config, 7);
    START_THREAD(fsm, SENSOR_CORE, config, 8);
    START_THREAD(buzzer, SENSOR_CORE, config, 6);
    START_THREAD(angularkalman, SENSOR_CORE, config, 7);
    START_THREAD(telemetry, SENSOR_CORE, config, 15);
    START_THREAD(hilsim, DATA_CORE, config, 1);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while(true) {
      THREAD_SLEEP(1000);
      // Serial.println(".");
    }
}

// ---- ENTRY FUNCS ----
void k_run() {
    // Default 256-byte RX buffer overflows when SFLP FIFO bursts arrive back-to-back.
    Serial.setRxBufferSize(4096);
    Serial.begin(921600);
    // while(!Serial);

    k_push_event(K_START_E);
    k_push_event(k_get_checksum_evt());

    // Wait until start signal (newline written to serial)
    k_wait_until('\n');
    k_ident();
    k_clear_inbuf();

    // Set up MIDAS
    k_midas_setup();
    k_init_sensordata();
    k_push_event(K_SETUP_DONE);

    // Start systems
    k_start();
}

void k_tick() {
    k_tick_data_report();
    k_handle_all_k_evt();
}
// KAL - Kamaji Abstraction Layer
#include "kal.h"

// ---- GLOBALS ----
MultipleLogSink<> sink;
RocketSystems systems{.log_sink = sink};

// ---- INIT SENSORS ----
void k_init_sensordata() {
    KRocketData* arg = &GLOBAL_DATA;

    // This list should run parallel to the list defined in data_logging. This macro will perform the opposite task,
    // mapping disc IDs to their locations in the struct.
    ASSOCIATE(arg->low_g, ID_LOWG);
    ASSOCIATE(arg->low_g_lsm, ID_LOWGLSM);
    ASSOCIATE(arg->high_g, ID_HIGHG);
    ASSOCIATE(arg->barometer, ID_BAROMETER);
    ASSOCIATE(arg->continuity, ID_CONTINUITY);
    ASSOCIATE(arg->voltage, ID_VOLTAGE);
    ASSOCIATE(arg->gps, ID_GPS);
    ASSOCIATE(arg->magnetometer, ID_MAGNETOMETER);
    ASSOCIATE(arg->orientation, ID_ORIENTATION);
    ASSOCIATE(arg->fsm_state, ID_FSM);
    ASSOCIATE(arg->kalman, ID_KALMAN);
    ASSOCIATE(arg->pyro, ID_PYRO);
}

// ---- Kamaji Thread ----
DECLARE_THREAD(hilsim, void*arg) {
    uint8_t header[5];
    uint8_t crc_buf[2];
    uint32_t timestamp;
    uint8_t discriminant;
    size_t data_size;
    uint8_t data_buf[128];
    uint16_t crc;
    
    while (true) {
        int delim;
        do {
            k_tick();
            delim = Serial.read();
            THREAD_SLEEP(1);

        } while(delim != '$' && delim != '#');

        if(delim == '#') {
          if (!k_read_exact(data_buf, 16, 10)) continue;

          // Sys message is 14 byte data, 2 byte crc  
          memcpy(&crc, data_buf + 14, sizeof(crc_buf));
          k_handle_sys_msg(data_buf, crc);
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
    // real midas setup
    INIT_SYSTEM(systems.sensors.low_g);
    INIT_SYSTEM(systems.sensors.orientation);
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
    INIT_SYSTEM(systems.tlm);

    Serial.println("Starting Systems...");
    RocketSystems* config = &systems;

    // START_THREAD(logger, DATA_CORE, config, 15);
    START_THREAD(orientation, SENSOR_CORE, config, 10);
    START_THREAD(accelerometers, SENSOR_CORE, config, 13);
    START_THREAD(barometer, SENSOR_CORE, config, 12);
    START_THREAD(gps, SENSOR_CORE, config, 8);
    START_THREAD(voltage, SENSOR_CORE, config, 9);
    START_THREAD(pyro, SENSOR_CORE, config, 14);
    START_THREAD(magnetometer, SENSOR_CORE, config, 11);
    START_THREAD(cam, SENSOR_CORE, config, 16);
    START_THREAD(kalman, SENSOR_CORE, config, 7);
    START_THREAD(fsm, SENSOR_CORE, config, 8);
    START_THREAD(buzzer, SENSOR_CORE, config, 6);
    START_THREAD(telemetry, SENSOR_CORE, config, 15);
    START_THREAD(hilsim, DATA_CORE, nullptr, 1);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while(true) {
      THREAD_SLEEP(1000);
      // Serial.println(".");
    }
}

// ---- ENTRY FUNCS ----
void k_run() {
    Serial.begin(115200);
    k_midas_setup();
    k_init_sensordata();
    k_push_event(K_SETUP_DONE);
    k_start();
}

void k_tick() {
    k_tick_data_report();
    k_handle_all_k_evt();
}
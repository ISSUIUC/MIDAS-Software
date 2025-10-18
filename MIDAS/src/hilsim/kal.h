// KAL - Kamaji Abstraction Layer
#include "esp_rom_crc.h"
#include "data_logging.h"
#include "log_format.h"
#include "systems.h"
#include "kal_rocket.h"

#define EVENT_STACK_SIZE 8

#pragma pack(push, 1)
typedef struct {uint32_t ts_us; uint8_t disc;} k_hdr;
#pragma pack(pop)

typedef struct SensorMapping {
    size_t struct_size;
    void* struct_map;
};

typedef struct {
    uint32_t timestamp;
    uint8_t size;
    uint8_t buf[32];
} k_event_t;

SensorMapping MAP[READING_DISC_COUNT-1];
k_event_t EVENT_STACK[EVENT_STACK_SIZE];
uint8_t st_remaining = 0;

#define ASSOCIATE(ty, id) MAP[id] = SensorMapping{sizeof(ty), &(ty)}

// Note: crc poly 0x1021
// Note: crc check is little endian
bool k_crc_check(const k_hdr* header, const uint8_t* payload, uint16_t size, uint16_t crc_cmp) {
    uint16_t crc = 0xFFFF;
    crc = esp_rom_crc16_le(crc, (const uint8_t*) header, sizeof(header));
    crc = esp_rom_crc16_le(crc, payload, size);
    return crc == crc_cmp;
}

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

void k_read_into_sensordata(uint8_t disc, uint8_t* data) {
    SensorMapping cur_map = MAP[disc - 1];
    memcpy(cur_map.struct_map, data, cur_map.struct_size);
}

size_t k_get_discriminant_size(uint8_t disc) {
    return MAP[disc-1].struct_size;
}

void k_set_timestamp(uint32_t ts) {
    return;
}

// Packet: $ uint32_t:timestamp uint8_t:disc uint8_t*:data uint16_t:crc
// Returns true if the reading was accepted
bool k_handle_reading(uint32_t ts, uint8_t disc, uint8_t* data, size_t data_size, uint16_t crc) {
    k_hdr header = {ts, disc};
    if(k_crc_check(&header, data, data_size, crc)) {
        k_read_into_sensordata(disc, data);
        k_set_timestamp(ts);
        return true;
    }

    return false;
}

void k_push_event(k_event_t evt) {
    if(st_remaining >= EVENT_STACK_SIZE - 1) {
        return; // Cannot allocate the event.
    }
    EVENT_STACK[st_remaining] = evt;
    st_remaining++;
}

void handle_all_k_evt() {
    while (st_remaining) {
        --st_remaining;
        handle_k_evt(EVENT_STACK[st_remaining]);
    }
}

void handle_k_evt(k_event_t evt) {
    Serial.print("%");
    Serial.print(evt.timestamp);
    Serial.print(evt.size);
    Serial.println((char*)evt.buf);
}

// Commands to communicate back to the streamer
void k_log(char* log_message, size_t len) {
    // +1 to handle \0
    k_event_t evt = {0, len+1, 0};
    memcpy(evt.buf, log_message, len+1);
    k_push_event(evt);
}

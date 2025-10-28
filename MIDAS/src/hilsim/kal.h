// KAL - Kamaji Abstraction Layer
#include "esp_rom_crc.h"
#include "data_logging.h"
#include "log_format.h"
#include "systems.h"
#include "kal_rocket.h"

#define EVENT_STACK_SIZE 8
#define MAX_DATA_REPORTS 16

#pragma pack(push, 1)
typedef struct {uint32_t ts_us; uint8_t disc;} k_hdr;
#pragma pack(pop)

typedef struct SensorMapping {
    size_t struct_size;
    void* struct_map;

    uint32_t report_interval = 0;
    unsigned long last_report_tick = 0;
};

// Non-halting exceptions
void k_EVENTOVERFLOW() {
    // Too many events allocated
    Serial.println("!EO");
}

void k_REPORTOVERFLOW() {
    // Too many reports enabled
    Serial.println("!RO");
}

void k_INVALIDINSTR() {
    // Invalid system instruction
    Serial.println("!II");
}

typedef struct {
    unsigned long timestamp;
    uint8_t size;
    uint8_t buf[128];
} k_event_t;

SensorMapping MAP[READING_DISC_COUNT-1];
ReadingDiscriminant data_reports[MAX_DATA_REPORTS];
k_event_t EVENT_STACK[EVENT_STACK_SIZE];

uint8_t st_remaining = 0;
uint8_t data_report_top = 0;

#define ASSOCIATE(ty, id) MAP[id] = SensorMapping{sizeof(ty), &(ty)}

// Note: crc poly 0x1021
// Note: crc check is little endian
bool k_crc_check(const k_hdr* header, const uint8_t* payload, uint16_t size, uint16_t crc_cmp) {
    uint16_t crc = 0xFFFF;
    crc = esp_rom_crc16_le(crc, (const uint8_t*) header, sizeof(header));
    crc = esp_rom_crc16_le(crc, payload, size);
    return crc == crc_cmp;
}

bool k_crc_check_sys(const uint8_t* payload, uint16_t crc_cmp) {
    uint16_t crc = 0xFFFF;
    crc = esp_rom_crc16_le(crc, payload, 14);
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
        k_EVENTOVERFLOW();
        return; // Cannot allocate the event.
    }
    EVENT_STACK[st_remaining] = evt;
    st_remaining++;
}

void k_handle_k_evt(k_event_t evt) {
    Serial.print("%");
    Serial.print(evt.timestamp);
    Serial.print(evt.size);
    Serial.println((char*)evt.buf);
}

void k_handle_all_k_evt() {
    while (st_remaining) {
        --st_remaining;
        k_handle_k_evt(EVENT_STACK[st_remaining]);
    }
}

// Commands to communicate back to the streamer
void k_log(char* log_message, size_t len) {
    // +1 to handle \0
    k_event_t evt = {0, (uint8_t)len+2, "@"};
    memcpy(evt.buf + 1, log_message, len+1);
    k_push_event(evt);
}

// Data reporting
void k_enable_data_report(ReadingDiscriminant sens_id, uint32_t report_interval) {
    if(data_report_top >= MAX_DATA_REPORTS - 1) {
        k_REPORTOVERFLOW();
        return;
    }
    data_reports[data_report_top] = sens_id;
    ++data_report_top;
    MAP[sens_id-1].report_interval = report_interval;
}

void k_tick_data_report() {
    unsigned long cur_t = millis();
    for(uint8_t i = 0; i < data_report_top; i++) {
        SensorMapping s = MAP[data_reports[i] - 1];

        if(s.report_interval) {
            if(cur_t - s.last_report_tick > s.report_interval) {
                k_event_t s_event = {cur_t, (uint8_t)s.struct_size, 0};
                memcpy(s_event.buf, s.struct_map, s.struct_size);
                k_push_event(s_event);
            }
        }
    }
}


enum sys_instr_t {
    RESERVED = 0,
    REPORT_EN = 1,
};

// System message: instruct the Kamaji driver to do something
// # <INSTR> <data[13]>
// INSTR
// - 0x00 --> reserved
// - 0x01 --> REPORT_EN
void k_handle_sys_msg(uint8_t* data, uint16_t crc) {
    sys_instr_t instr = (sys_instr_t) data[0];

    switch(instr) {
        case (sys_instr_t::REPORT_EN):
            {
                ReadingDiscriminant disc = (ReadingDiscriminant) data[1];
                uint32_t report_intv = 0;
                memcpy(&report_intv, data + 2, sizeof(uint32_t));
                k_enable_data_report(disc, report_intv);
            }
            break;
        default:
            k_INVALIDINSTR();
            return;
    }

    Serial.println("%OK");
}

// High level entries
void k_setup() {
    k_init_sensordata();
}

void k_tick() {
    k_tick_data_report();
    k_handle_all_k_evt();
}
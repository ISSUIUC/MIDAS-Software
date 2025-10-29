#pragma once
#include "data_logging.h"
#include "log_format.h"
#include "kal_error.h"
#include "kal_events.h"

// ---- CONFIGURATION ----
#define MAX_DATA_REPORTS 16

// ---- TYPES ----
struct SensorMapping {
    size_t struct_size;
    void* struct_map;

    uint32_t report_interval = 0;
    unsigned long last_report_tick = 0;
};

#define ASSOCIATE(ty, id) MAP[id-1] = SensorMapping{sizeof(ty), &(ty)}

inline SensorMapping MAP[READING_DISC_COUNT-1]; // Sensor data map
inline ReadingDiscriminant data_reports[MAX_DATA_REPORTS]; // Reports stack
inline uint8_t data_report_top = 0; // data_reports top pointer

inline void k_read_into_sensordata(uint8_t disc, uint8_t* data) {
    SensorMapping cur_map = MAP[disc - 1];
    memcpy(cur_map.struct_map, data, cur_map.struct_size);
}

inline size_t k_get_discriminant_size(uint8_t disc) {
    return MAP[disc-1].struct_size;
}

inline void k_set_timestamp(uint32_t ts) {
    return;
}

// Data reporting
inline void k_enable_data_report(ReadingDiscriminant sens_id, uint32_t report_interval) {
    if(data_report_top >= MAX_DATA_REPORTS - 1) {
        k_REPORTOVERFLOW();
        return;
    }
    data_reports[data_report_top] = sens_id;
    ++data_report_top;
    MAP[sens_id-1].report_interval = report_interval;
}

inline void k_tick_data_report() {
    unsigned long cur_t = millis();
    for(uint8_t i = 0; i < data_report_top; i++) {
        uint8_t disc_id = (uint8_t)data_reports[i];
        ReadingDiscriminant disc = (ReadingDiscriminant) (disc_id-1);
        SensorMapping* s = &MAP[disc];
        if(s->report_interval) {

            if((cur_t - s->last_report_tick) > s->report_interval) {
                // The first byte of the event data will be the disc, so its size + 1 byte for disc
                k_event_t s_event = {cur_t, K_EVENT_TYPE::DATA_REPORT, (uint8_t) (s->struct_size + 1), 0};
                s->last_report_tick = cur_t;
                memcpy(s_event.buf, &disc_id, sizeof(uint8_t));
                memcpy(s_event.buf + 1, s->struct_map, s->struct_size);
                k_push_event(s_event);

            }
        }
    }
}

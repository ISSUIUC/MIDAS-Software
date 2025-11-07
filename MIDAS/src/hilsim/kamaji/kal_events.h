#pragma once
#include <cstdint>
#include "kal_error.h"
#include "log_checksum.h"

// ---- CONFIGURATION ----
#define EVENT_STACK_SIZE 8


// ---- IMPLEMENTATION ----
enum K_EVENT_TYPE {
    SYSTEM_MESSAGE = 0,
    DATA_REPORT = 1,
    LOG = 2,
};

typedef struct {
    unsigned long timestamp;
    K_EVENT_TYPE event_type; // 0--sys msg, 1--data report, 2--log
    uint8_t size;
    uint8_t buf[128];
} k_event_t;

inline uint8_t st_remaining = 0; // Head pointer for the EVENT_STACK
inline k_event_t EVENT_STACK[EVENT_STACK_SIZE];

// ---- EVT FUNCTIONS ----
inline void k_push_event(k_event_t evt) {
    if(st_remaining >= EVENT_STACK_SIZE - 1) {
        k_EVENTOVERFLOW();
        return; // Cannot allocate the event.
    }
    EVENT_STACK[st_remaining] = evt;
    st_remaining++;
}

inline void k_handle_k_evt(k_event_t evt) {

    uint8_t ts_buf[4];
    uint8_t event_type = (uint8_t) evt.event_type;
    memcpy(ts_buf, &evt.timestamp, sizeof(evt.timestamp));

    Serial.write('%');
    Serial.write(ts_buf, sizeof(evt.timestamp));
    Serial.write(event_type);
    Serial.write(&evt.size, sizeof(evt.size));
    Serial.write((char*)evt.buf, evt.size);
}


inline void k_handle_all_k_evt() {
    while (st_remaining) {
        --st_remaining;
        k_handle_k_evt(EVENT_STACK[st_remaining]);
    }
}

// Commands to communicate back to the streamer
inline void k_log(char* log_message, size_t len) {
    // +1 to handle \0
    k_event_t evt = {0, K_EVENT_TYPE::LOG, (uint8_t)len+2, "@"};
    memcpy(evt.buf + 1, log_message, len+1);
    k_push_event(evt);
}

inline k_event_t k_get_checksum_evt() {
    k_event_t evt;
    uint32_t checksum = LOG_CHECKSUM;
    memcpy(evt.buf + 1, &checksum, sizeof(uint32_t));
    evt.buf[0] = 'C';
    return evt;
}

// ---- PRE-DEFINED EVENTS ----
constexpr k_event_t K_START_E {0, K_EVENT_TYPE::SYSTEM_MESSAGE, 3, {'H', 'I', 'L'}};
constexpr k_event_t K_SETUP_DONE {0, K_EVENT_TYPE::SYSTEM_MESSAGE, 3, {'H', 'S', 'D'}};
// KAL - Kamaji Abstraction Layer
#pragma once
#include "systems.h"
#include "kamaji/kal_rocket.h"
#include "kamaji/kal_error.h"
#include "kamaji/kal_events.h"
#include "kamaji/kal_crc.h"
#include "kamaji/kal_sensordata.h"
#include "kamaji/kal_interface.h"

#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { Serial.println("init err"); while(true); } } while (0)

// ---- INIT SENSORS ----
void k_init_sensordata();

// ---- UTIL ----
static bool k_read_exact(uint8_t* dst, size_t n, unsigned long timeout_ms) {
  unsigned long start = millis();
  size_t got = 0;
  while (got < n) {
    int avail = Serial.available();
    if (avail > 0) {
      int r = Serial.readBytes(dst + got, n - got);
      got += (r > 0 ? (size_t)r : 0);
      continue;
    }
    if ((millis() - start) > timeout_ms) return false;
    THREAD_SLEEP(1); // yield
  }
  return true;
}

// ---- HANDLE SYS MESSAGES ----
// Packet: $ uint32_t:timestamp uint8_t:disc uint8_t*:data uint16_t:crc
// Returns true if the reading was accepted
inline bool k_handle_reading(uint32_t ts, uint8_t disc, uint8_t* data, size_t data_size, uint16_t crc) {
    k_hdr header = {ts, disc};
    if(k_crc_check(&header, data, data_size, crc)) {
        k_read_into_sensordata(disc, data);
        k_set_timestamp(ts);
        return true;
    }

    return false;
}


enum sys_instr_t {
    RESERVED = 0,
    REPORT_EN = 1,
    VERIFY_CHECKSUM = 2,
};

// System message: instruct the Kamaji driver to do something
// # <INSTR> <data[13]>
// INSTR
// - 0x00 --> reserved
// - 0x01 --> REPORT_EN
inline void k_handle_sys_msg(uint8_t* data, uint16_t crc) {
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
        case (sys_instr_t::VERIFY_CHECKSUM):
            {
                uint32_t checksum;
                memcpy(&checksum, data + 1, sizeof(uint32_t));
                if(checksum != LOG_CHECKSUM) {
                    k_INVALIDCHECKSUM();
                }
            }
            break;
        default:
            k_INVALIDINSTR();
            return;
    }

    Serial.write("%OK");
}

void k_wait_until(char sig) {
    char a;
    do {
        a = Serial.read();
    } while (a != sig);
}

void k_clear_inbuf() {
    while(Serial.available() > 0) {
        char tb = Serial.read();
    }
}

// ---- Kamaji Thread ----
void hilsim_thread(void* arg);

// Run the Kamaji process
[[noreturn]] void k_start();

// ---- ENTRY FUNCS ----
void k_run();
void k_tick();
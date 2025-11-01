#pragma once
#include <Arduino.h>

// Non-halting exceptions
inline void k_EVENTOVERFLOW() {
    // Too many events allocated
    Serial.write("!EO\n");
}

inline void k_REPORTOVERFLOW() {
    // Too many reports enabled
    Serial.write("!RO\n");
}

inline void k_INVALIDINSTR() {
    // Invalid system instruction
    Serial.write("!II\n");
}

inline void k_INVALIDCHECKSUM() {
    // Invalid checksum test
    Serial.write("!IC\n");
}


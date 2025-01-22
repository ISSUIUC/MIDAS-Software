#include "arduino_emulation.h"
#include <cstdint>
#include <iostream>

extern uint32_t global_ms;

uint32_t millis(){
    return global_ms;
}

void SerialPatch::begin(int baudrate) {
    (void) baudrate;
}

void SerialPatch::flush() {
    std::flush(std::cout);
}

SerialPatch Serial;

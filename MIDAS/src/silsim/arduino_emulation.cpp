#include "arduino_emulation.h"
#include <cstdint>
#include <iostream>

extern size_t global_ms;

unsigned long millis(){
    return global_ms;
}

void SerialPatch::begin(int baudrate) {}

SerialPatch Serial;

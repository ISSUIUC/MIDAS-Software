#pragma once
#include <iostream>
#include <cstdint>

uint32_t millis();

struct SerialPatch {
    template <typename T>
    void print(T t) {
        std::cout << t;
    }

    template <typename T>
    void println(T t) {
        std::cout << t << '\n';
    }

    void begin(int baudrate);
    void flush();
};

extern SerialPatch Serial;

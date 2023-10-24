#pragma once
#include <iostream>

uint32_t millis();

struct SerialPatch {
//    void println(const char* s);

    template <typename T>
    void print(T t) {
        std::cout << t;
    }

    template <typename T>
    void println(T t) {
        std::cout << t << '\n';
    }

    void begin(int baudrate);
};

extern SerialPatch Serial;
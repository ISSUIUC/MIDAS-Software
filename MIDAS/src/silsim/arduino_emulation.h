#pragma once
#include <iostream>

unsigned long millis();

struct SerialPatch {
    void println(const char* s);

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
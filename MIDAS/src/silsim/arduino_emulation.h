#pragma once
#include <iostream>

unsigned long millis();


struct SerialPatch {
    void println(const char* s);

    template <typename T>
    void print(T t) {
        std::cout << t;
    }

    template <typename T, typename... Args>
    void print(T t, Args... args) {
        std::cout << t;

        print(args...);
    }

    template <typename T>
    void println(T t) {
        std::cout << t << '\n';
    }

    template <typename T, typename... Args>
    void println(T t, Args... args) {
        std::cout << t;

        println(args...);
    }

    void begin(int baudrate);
};

extern SerialPatch Serial;
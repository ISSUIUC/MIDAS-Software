#pragma once

#ifndef SILSIM

#include <FreeRTOS.h>
#include <semphr.h>

template<typename T>
struct Mutex {
    StaticSemaphore_t mutex_buffer;
    SemaphoreHandle_t mutex_handle;

    T data;

    Mutex() {
        mutex_handle = xSemaphoreCreateMutexStatic(&mutex_buffer);
    }
};

#else

template<typename T>
class Mutex {

};

#endif
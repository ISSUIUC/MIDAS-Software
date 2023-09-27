#pragma once

#include "hal.h"

template<typename T>
struct Mutex {
private:
    StaticSemaphore_t mutex_buffer;
    SemaphoreHandle_t mutex_handle;

    T data;

public:
    Mutex() = delete;

    explicit Mutex(T data) : mutex_buffer() {
        mutex_handle = xSemaphoreCreateMutexStatic(&mutex_buffer);
    }

    T read() {
        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        T ret = data;
        xSemaphoreGive(mutex_handle);
        return ret;
    }

    void write(T value) {
        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        data = value;
        xSemaphoreGive(mutex_handle);
    }

    // todo lock/unlock?
};

template<typename T, uint64_t length = QUEUE_LENGTH>
class Queue {
private:
    StaticQueue_t queue_area;
    uint8_t buffer[sizeof(T) * length];
    QueueHandle_t queue;

public:
    Queue() {
        queue = xQueueCreateStatic(length, sizeof(T), buffer, &queue_area);
    }

    void send(T value) {
        xQueueSendToBack(queue, &value, MUTEX_TIMEOUT);
    }

    bool receive(T* out) {
        return xQueueReceive(queue, out, MUTEX_TIMEOUT);
    }
};

#undef MUTEX_TIMEOUT

#define STACK_SIZE 1024

#define SENSOR_CORE ((BaseType_t) 0)
#define DATA_CORE ((BaseType_t) 1)

#define DECLARE_THREAD(name, param) [[noreturn]] void name##_thread(param)
#define START_THREAD(name, core, arg) StaticTask_t name##_task;                \
                                      unsigned char name##_stack[STACK_SIZE];            \
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) name##_thread), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + 1, name##_stack, &name##_task, core)
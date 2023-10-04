#pragma once

#include <cstdio>
#include <cstdint>
#include <queue>

#include "fiber.h"

#define tskIDLE_PRIORITY (0)

#define pdMS_TO_TICKS(ms) (ms)

void threadYield();
void threadSleep(int32_t time_ms);

typedef int StaticTask_t;
typedef int BaseType_t;
typedef void (*TaskFunction_t)(void*);

void xTaskCreateStaticPinnedToCore(TaskFunction_t thread_fn, const char* name, size_t stack_size, void* argument, int priority, unsigned char* stack, StaticTask_t* task_handle, BaseType_t core);

typedef int StaticSemaphore_t;

struct SemaphoreHandle_s {
public:
    SemaphoreHandle_s() : locked(false) { };

    void lock() {
        while (locked) {
            threadYield();
        }
    }

    void unlock() { locked = false; }

private:
    volatile bool locked;
};

typedef SemaphoreHandle_s* SemaphoreHandle_t;
typedef size_t TickType_t;

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* buffer);
bool xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout);

typedef int StaticQueue_t;


void vTaskDelay(int32_t time);
void vTaskDelete(void* something_probably);

void begin_silsim();
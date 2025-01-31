#pragma once

#include <cstdio>
#include <cstdint>
#include <queue>
#include <cstring>
#include <iostream>

#include "silsim/fiber.h"

#define LOW (0x0)
#define HIGH (0x1)
#define OUTPUT (0x3)

#define tskIDLE_PRIORITY (128)

#define pdMS_TO_TICKS(ms) (ms)
#define pdTICKS_TO_MS(ticks) (ticks)
#define xTaskGetTickCount() (millis())

uint32_t millis();

void threadYield();
void threadSleep(int32_t time_ms);
void delay(unsigned long ms);
void emu_busy_wait(size_t ms);

uint32_t ledcWriteTone(uint8_t channel, uint32_t frequency);
void ledcAttachPin(uint8_t pin, uint8_t channel);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);

typedef int StaticTask_t;
typedef int BaseType_t;
typedef void (*TaskFunction_t)(void*);

void xTaskCreateStaticPinnedToCore(TaskFunction_t thread_fn, const char* name, size_t stack_size, void* argument, int priority, unsigned char* stack, StaticTask_t* task_handle, BaseType_t core);

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

#define portMAX_DELAY ((TickType_t) -1)

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* buffer);
bool xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout);
bool xSemaphoreGive(SemaphoreHandle_t semaphore);

class StaticQueue_t {
public:
    StaticQueue_t() : item_size(0), max_count(0), buffer(nullptr) { };

    StaticQueue_t* initialize(size_t max_count_, size_t item_size_, uint8_t* buffer_) {
        item_size = item_size_;
        max_count = max_count_;
        buffer = buffer_;
        return this;
    }

    bool push(void* item) {
        std::memcpy(&buffer[tail_idx * item_size], item, item_size);
        tail_idx++;
        if (tail_idx == max_count) {
            tail_idx = 0;
        }
        if (count == max_count) {
            head_idx++;
            if (head_idx == max_count) {
                head_idx = 0;
            }
        } else {
            count++;
        }
        return true;
    }

    bool pop(void* item) {
        if (count == 0) {
            return false;
        }
        std::memcpy(item, &buffer[head_idx * item_size], item_size);
        head_idx++;
        if (head_idx == max_count) {
            head_idx = 0;
        }
        count--;
        return true;
    }

private:
    size_t count = 0;     // The number of items currently in the Queue.
    size_t head_idx = 0;  // The index to for the next pop to read from.
    size_t tail_idx = 0;  // The index to for the next push to write to.

    size_t item_size;
    size_t max_count;
    uint8_t* buffer;
};
typedef StaticQueue_t* QueueHandle_t;

#define xQueueCreateStatic(length, item_size, buffer, queue) ((queue)->initialize(length, item_size, buffer))
#define xQueueSendToBack(queue, value_ptr, timeout) ((queue)->push(value_ptr))
#define xQueueReceive(queue, store_ptr, timeout) ((queue)->pop(store_ptr))

#define errQUEUE_FULL (false)

void vTaskDelay(int32_t time);
void vTaskDelete(void* something_probably);

void begin_silsim();
void silsim_step_time(uint32_t ms, uint32_t ticks_per_ms);
double silsim_current_time();

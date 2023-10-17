#include "silsim/emulation.h"

#include <iostream>
#include <vector>
#include <chrono>

struct ThreadInfo {
public:
    explicit ThreadInfo(const char* name, FiberHandle handle) : _name(name), fiber_handle(handle) {}

    const char* _name;
    FiberHandle fiber_handle;
};

struct ThreadManager {
public:
    ThreadManager() : threads(), next_thread(1) {}

    std::vector<ThreadInfo> threads;
    size_t next_thread;

    bool debug = false;

    void yield() {
        if (threads.empty()) {
            return;
        }
        if (next_thread >= threads.size()) {
            next_thread = 0;
        }
        ThreadInfo* switch_to = &threads[next_thread];
        next_thread++;
        if (debug) std::cout << "Switching to " << switch_to->_name << std::endl;
        EmuSwitchToFiber(switch_to->fiber_handle);
    }
};

ThreadManager thread_manager;

double time_since_start = 0.0;

void threadYield() { thread_manager.yield(); }

void threadSleep(int32_t time_ms) {
    double start = time_since_start;
    double wait = time_ms / 1000.0;
    while (true) {
        double curr = time_since_start;
        if (curr >= start + wait) {
            return;
        }

        threadYield();
    }
}

void createThread(const char* name, void fn(void*), size_t stack, void* arg) {
    FiberHandle handle = EmuCreateFiber(stack, fn, arg);
    thread_manager.threads.emplace_back(name, handle);
}


void xTaskCreateStaticPinnedToCore(TaskFunction_t thread_fn, const char* name, size_t stack_size, void* argument,
                                   int priority, unsigned char* stack, StaticTask_t* task_handle, BaseType_t core) {
    createThread(name, thread_fn, stack_size, argument);
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* buffer) {
    return new SemaphoreHandle_s();
}

bool xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout) {
    semaphore->lock();
    return true;
}

bool xSemaphoreGive(SemaphoreHandle_t semaphore) {
    semaphore->unlock();
    return true;
}

[[noreturn]] void time_is_real(void* arg) {
    time_since_start = 0.0;
    auto start = std::chrono::system_clock::now();
    while (true) {
        auto curr = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = curr - start;
        time_since_start = elapsed_seconds.count();
        threadYield();
    }
}


void begin_silsim() {
    thread_manager.threads.emplace_back("main", EmuConvertThreadToFiber());
    createThread("time is real", time_is_real, 4096, nullptr);
}

void vTaskDelete(void* something_probably) {

}

void vTaskDelay(int32_t time) {
    threadSleep(time);
}


void SerialPatch::println(const char* s) { std::cout << s << '\n'; }

void SerialPatch::begin(int baudrate) {}

SerialPatch Serial;

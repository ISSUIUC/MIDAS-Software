#include "silsim/emulation.h"

#include <vector>
#include <chrono>
#include <array>

size_t global_ms = 0;

struct ThreadInfo {
public:
    explicit ThreadInfo(const char* name, FiberHandle handle, BaseType_t core) : name(name), fiber_handle(handle), core(core) {}

    const char* name;
    FiberHandle fiber_handle;
    BaseType_t core;
};

struct ThreadManager {
public:
    ThreadManager() : threads(), next_thread(1), core_quantums() {}

    std::vector<ThreadInfo> threads;
    size_t next_thread;
    std::array<size_t, 2> core_quantums;

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

        if(switch_to->core == -1){ //silsim manager thread
            if (debug) std::cout << "Switching to " << switch_to->name << std::endl;
            EmuSwitchToFiber(switch_to->fiber_handle);
        } else if(core_quantums.at(switch_to->core) <= global_ms){
            core_quantums[switch_to->core] = global_ms;
            if (debug) std::cout << "Switching to " << switch_to->name << std::endl;
            EmuSwitchToFiber(switch_to->fiber_handle);
        }
    }
};

ThreadManager thread_manager;


void threadYield() { thread_manager.yield(); }

void threadSleep(int32_t time_ms) {
    size_t start = global_ms;
    while (true) {
        if (global_ms >= start + time_ms) {
            return;
        }

        threadYield();
    }
}

void silsimStepTime() {
    global_ms++;
    threadYield();
}

void createThread(const char* name, void fn(void*), size_t stack, void* arg, BaseType_t core) {
    FiberHandle handle = EmuCreateFiber(stack, fn, arg);
    thread_manager.threads.push_back(ThreadInfo(name, handle, core));
}


void xTaskCreateStaticPinnedToCore(TaskFunction_t thread_fn, const char* name, size_t stack_size, void* argument,
                                   int priority, unsigned char* stack, StaticTask_t* task_handle, BaseType_t core) {
    createThread(name, thread_fn, stack_size, argument, core);
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

void begin_silsim() {
    thread_manager.threads.emplace_back("main", EmuConvertThreadToFiber(), -1);
}

void vTaskDelete(void* something_probably) {

}

void vTaskDelay(int32_t time) {
    threadSleep(time);
}

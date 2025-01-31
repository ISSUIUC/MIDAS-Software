#include "silsim/emulation.h"

#include <vector>
#include <chrono>
#include <array>
#include <map>

struct ThreadInfo {
public:
    explicit ThreadInfo(const char* name, FiberHandle handle, BaseType_t core) : name(name), fiber_handle(handle), core(core) {}

    const char* name;
    FiberHandle fiber_handle;
    BaseType_t core;
};

struct Core {
    std::vector<ThreadInfo> threads;
    size_t next_idx;
    size_t current_quantum;
    size_t core_idx;
};

class ThreadManager {
    size_t core_idx;
    std::vector<Core> cores;

    uint32_t global_ms;

    bool debug = false;

public:
    ThreadManager(): core_idx(0), cores(3), global_ms(0) {}

    void add_thread(ThreadInfo t){
        cores.at(t.core).threads.push_back(t);
    }

    void yield() {
        while (true) {
            Core& core = cores[core_idx];
            if (core.next_idx == core.threads.size()) {
                core.next_idx = 0;
                core_idx = (core_idx + 1) % cores.size();
                continue;
            }
            if (core.current_quantum <= global_ms) {
                core.current_quantum = global_ms;
            } else {
                core_idx = (core_idx + 1) % cores.size();
                continue;
            }

            ThreadInfo& thread = core.threads[core.next_idx];
            core.next_idx++;
            if (debug) {
                std::cout << "Switching to " << thread.name << std::endl;
            }
            EmuSwitchToFiber(thread.fiber_handle);
            break;
        }
    }

    void busy_wait_current_core(size_t ms){
        cores[core_idx].current_quantum += ms;
    }

    void set_time(uint32_t time_ms) {
        if (time_ms > global_ms) {
            global_ms = time_ms;
        }
    }

    uint32_t get_time() const {
        return global_ms;
    }
};

ThreadManager thread_manager;

void silsim_step_time(uint32_t ms, uint32_t ticks_per_ms) {
    for (uint32_t i = 0; i < ms * ticks_per_ms; i++) {
        threadYield();
    }
    thread_manager.set_time(thread_manager.get_time() + ms);
}

double silsim_current_time() {
    return ((double) thread_manager.get_time()) / 1000.0;
}

void threadYield() { thread_manager.yield(); }

void threadSleep(int32_t time_ms) {
    uint32_t start = thread_manager.get_time();
    while (true) {
        if (thread_manager.get_time() >= start + time_ms) {
            return;
        }

        threadYield();
    }
}

void delay(unsigned long ms){
    threadSleep(ms);
}

void emu_busy_wait(size_t ms){
    thread_manager.busy_wait_current_core(ms);
}

void createThread(const char* name, void fn(void*), size_t stack, void* arg, BaseType_t core) {
    FiberHandle handle = EmuCreateFiber(stack, fn, arg);
    thread_manager.add_thread(ThreadInfo(name, handle, core));
}


void xTaskCreateStaticPinnedToCore(TaskFunction_t thread_fn, const char* name, size_t stack_size, void* argument,
                                   int priority, unsigned char* stack, StaticTask_t* task_handle, BaseType_t core) {
    (void) priority, (void) stack, (void) task_handle;
    createThread(name, thread_fn, stack_size, argument, core);
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* buffer) {
    (void) buffer;
    return new SemaphoreHandle_s();
}

bool xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout) {
    (void) timeout;
    semaphore->lock();
    return true;
}

bool xSemaphoreGive(SemaphoreHandle_t semaphore) {
    semaphore->unlock();
    return true;
}

void begin_silsim() {
    thread_manager.add_thread(ThreadInfo{"main", EmuConvertThreadToFiber(), 2});
}

void vTaskDelete(void* something_probably) {
    (void) something_probably;
}

void vTaskDelay(int32_t time) {
    threadSleep(time);
}

uint8_t channels[256];

uint32_t ledcWriteTone(uint8_t channel, uint32_t frequency) {
    std::cout << "Writing tone with frequency " << (int) frequency << " onto channel " << (int) channel << "." << std::endl;
    return 0;
}

void ledcAttachPin(uint8_t pin, uint8_t channel) {
    channels[channel] = pin;
}

void pinMode(uint8_t pin, uint8_t mode) {
    (void) pin, (void) mode;
}

void digitalWrite(uint8_t pin, uint8_t value) {
    (void) pin, (void) value;
}

uint32_t millis() {
    return thread_manager.get_time();
}

void SerialPatch::begin(int baudrate) {
    (void) baudrate;
}

void SerialPatch::flush() {
    std::flush(std::cout);
}

SerialPatch Serial;

#pragma once

#ifndef SILSIM

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define MUTEX_TIMEOUT ((TickType_t) 100000)

/**
 * The default maximum length for a Queue.
 */
#define QUEUE_LENGTH 100

/**
 * A mutex which holds a single value.
 *
 * @tparam T The type of the value to hold. Should have copy semantics.
 */
template<typename T>
struct Mutex {
private:
    StaticSemaphore_t mutex_buffer;
    SemaphoreHandle_t mutex_handle;

    T data;

public:
    Mutex() = delete;

    /**
     * Initializes the mutex with an initial contained value.
     *
     * @param data The initial value in the mutex.
     */
    explicit Mutex(T data) : mutex_buffer() {
        mutex_handle = xSemaphoreCreateMutexStatic(&mutex_buffer);
    }

    /**
     * Read the current value in the mutex (will spin until it acquires the lock).
     *
     * @return The value in the mutex.
     */
    T read() {
        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        T ret = data;
        xSemaphoreGive(mutex_handle);
        return ret;
    }
    /**
     * Update the value in the mutex (will spin until it acquires the lock).
     *
     * @param value What to update the mutex to.
     */
    void write(T value) {
        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        data = value;
        xSemaphoreGive(mutex_handle);
    }

    // todo lock/unlock?
};

/**
 * A Thread-safe Queue containing a single data type.
 *
 * @tparam T The data type stored in the queue.
 * @tparam length The maximum length of the queue. Defaults to QUEUE_LENGTH.
 */
template<typename T, uint64_t length = QUEUE_LENGTH>
class Queue {
private:
    // todo probably have the Queue Store the timestamps too
    StaticQueue_t queue_area;
    uint8_t buffer[sizeof(T) * length];
    QueueHandle_t queue;

public:
    Queue() {
        queue = xQueueCreateStatic(length, sizeof(T), buffer, &queue_area);
    }

    /**
     * Put a value in the queue. If the queue is full or timed out, do nothing.
     *
     * @param value The value to put in the queue.
     */
    void send(T value) {
        xQueueSendToBack(queue, &value, MUTEX_TIMEOUT);
    }

    /**
     * Fetch a value from the queue.
     *
     * @param out Where to put the value from the queue. It is undefined if there is no value in the queue, or if the queue timed out.
     * @return True if there was a value in the queue that was read into `out`, false otherwise.
     */
    bool receive(T* out) {
        return xQueueReceive(queue, out, MUTEX_TIMEOUT);
    }
};

#undef MUTEX_TIMEOUT

/**
 *  The size of a stack, in bytes.
 */
#define STACK_SIZE 1024

/**
 * The two cores we have: 
 * SENSOR_CORE will hold all of the sensor tasks writing to the struct
 * DATA_CORE will have all of the tasks reading from the struct for data logging, along with GPS
*/
#define SENSOR_CORE ((BaseType_t) 0)
#define DATA_CORE ((BaseType_t) 1)

/**
 * Macro for declaring the functionality of a thread.
 *
 * @param name The name of the task.
 * @param param The single parameter to input into the thread.
 */
#define DECLARE_THREAD(name, param) [[noreturn]] void name##_thread(param)

/**
 * Macro for creating the thread itself. This is a statement and has no return value.
 * Never put this in a scope that will end before the thread should.
 *
 * @param name name of the thread
 * @param core core for the task to be pinned to
 * @param arg config arguments for the task to take in
 */
#define START_THREAD(name, core, arg) StaticTask_t name##_task;                \
                                      unsigned char name##_stack[STACK_SIZE];            \
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) name##_thread), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + 1, name##_stack, &name##_task, core)

#define THREAD_SLEEP(millis) (vTaskDelay((millis) / portTICK_PERIOD_MS))

/**
 * Parameters for xTaskCreateStaticPinnedToCore are as follows in parameter order:
 * Function to be run by the thread, this contains a `while(true)` loop
 * Name of thread
 * Size of the stack for each thread in words (1 word = 4 bytes)
 * Arguments to be passed into the function, this will generally eb the config file
 * Priority of the task, in allmost all cases, this will be the idle priority plus one
 * The actual stack memory to use
 * A handle to reference the task with
 * The core to pin the task to
 */

#else

#include "silsim/emulation.h"

#endif
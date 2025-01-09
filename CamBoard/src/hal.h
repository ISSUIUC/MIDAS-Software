#pragma once

// the only thing #ifdef we need, theoretically

#include <Arduino.h>

#include "FreeRTOSConfig.h"

#include "Mutex.h"
#include "Queue.h"

/**
 * The size of a thread stack, in bytes.
 */
#define STACK_SIZE 8192

/**
 * The ESP32 has two physical cores, which will each be dedicated to one group of threads.
 * The SENSOR_CORE runs the threads which write to the sensor_data struct (mostly sensor polling threads).
 */
#define SENSOR_CORE ((BaseType_t) 0)

/**
 * The ESP32 has two physical cores, which will each be dedicated to one group of threads.
 * The DATA_CORE runs the GPS thread, as well as the threads which read from the sensor_data struct (e.g. SD logging).
 */
#define DATA_CORE ((BaseType_t) 1)

/**
 * Macro for declaring a thread. Creates a function with the name suffixed with `_thread`, annotated with [[noreturn]].
 *
 * @param name The name of the task.
 * @param param The single parameter to input into the thread.
 */
#define DECLARE_THREAD(name, param) [[noreturn]] void name##_thread(param)

/**
 * Macro for creating and starting a thread declared with `DECLARE_THREAD`. This is a statement and has no return value.
 * Never put this in a scope that will end before the thread should.
 *
 * @param name Name of the thread.
 * @param core Core for the task to be pinned to, either `SENSOR_CORE` or `DATA_CORE`.
 * @param arg Argument passed in to the `param` argument of `DECLARE_THREAD`.
 * @param prio Priority of the thread.
 */
#define START_THREAD(name, core, arg, prio) StaticTask_t name##_task;                \
                                      static unsigned char name##_stack[STACK_SIZE];            \
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) name##_thread), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + prio, name##_stack, &name##_task, core)
/*
 * Parameters for xTaskCreateStaticPinnedToCore are as follows in parameter order:
 *  - Function to be run by the thread, this contains a `while(true)` loop
 *  - Name of thread
 *  - Size of the stack for each thread in words (1 word = 4 bytes)
 *  - Arguments to be passed into the function, this will generally eb the config file
 *  - Priority of the task, in allmost all cases, this will be the idle priority plus one
 *  - The actual stack memory to use
 *  - A handle to reference the task with
 *  - The core to pin the task to
 */

/**
 * @brief Delays the running thread.
 * @param millis The time to delay in milliseconds.
*/
#define THREAD_SLEEP(millis) vTaskDelay(pdMS_TO_TICKS(millis))
#pragma once

#ifdef SILSIM
#include "silsim/emulation.h"
#else
#include <Arduino.h>
#include "FreeRTOSConfig.h"

#include "hardware/Mutex.h"
#include "hardware/Queue.h"
#endif

/**
 *  The size of a thread stack, in bytes.
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
                                      static unsigned char name##_stack[STACK_SIZE];            \
                                      xTaskCreateStaticPinnedToCore(((TaskFunction_t) name##_thread), #name, STACK_SIZE, arg, tskIDLE_PRIORITY + 1, name##_stack, &name##_task, core)
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

/**
 * delays a task for a certain amount of time in milliseconds
 * @param millis the time to delay in milliseconds
*/
#define THREAD_SLEEP(millis) vTaskDelay(pdMS_TO_TICKS(millis))
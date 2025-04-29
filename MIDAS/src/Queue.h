#pragma once

#ifdef SILSIM
#include "silsim/emulation.h"
#else
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#endif

/**
 * @brief The default maximum length for a Queue.
 */
#define QUEUE_LENGTH 128

/**
 * @brief A Thread-safe Queue containing a single data type.
 *
 * @tparam T The data type stored in the queue.
 * @tparam length The maximum length of the queue. Defaults to QUEUE_LENGTH.
 */
template<typename T, uint64_t length = QUEUE_LENGTH>
class Queue {
// private:
public:
    // todo probably have the Queue Store the timestamps too
    uint8_t queue_area[sizeof(StaticQueue_t) + 64];
    uint8_t buffer[sizeof(T) * length]{};
    QueueHandle_t queue{};

public:
    Queue() {
        queue = xQueueCreateStatic(length, sizeof(T), buffer, (StaticQueue_t*) &queue_area[32]);
        // configASSERT(mutex_handle);
    }
    Queue(const Queue&) = delete;
    Queue(Queue&&) = delete;

    /**
     * @brief Put a value in the queue. If the queue is full or timed out, do nothing.
     *
     * @param value The value to put in the queue.
     */
    void send(T value) {
        if(xQueueSendToBack(queue, &value, 0) == errQUEUE_FULL){
            //Serial.print("FULL QUEUE");
            //Serial.println(pcTaskGetName(nullptr));
        }
    }

    /**
     * @brief Fetch a value from the queue.
     *
     * @param out Where to put the value from the queue. It is undefined if there is no value in the queue, or if the queue timed out.
     * @return True if there was a value in the queue that was read into `out`, false otherwise.
     */
    bool receive(T* out) {
        return xQueueReceive(queue, out, 0);
    }
};

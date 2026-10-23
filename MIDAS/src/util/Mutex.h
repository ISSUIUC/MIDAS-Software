#ifdef SILSIM
#include "silsim/emulation.h"
#else
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#endif

#define MUTEX_TIMEOUT pdMS_TO_TICKS(100)


/**
 * @brief A mutex which holds a single value.
 *
 * @tparam T The type of the value to hold. Should have copy semantics.
 */
template<typename T>
struct Mutex {
private:
//    StaticSemaphore_t mutex_buffer;
    uint8_t mutex_buffer[sizeof(StaticSemaphore_t) + 64]{};
    SemaphoreHandle_t mutex_handle{};
    T data;
    SemaphoreHandle_t check{};

public:
    Mutex() = delete;
    Mutex(const Mutex&) = delete;
    Mutex(Mutex&&) = delete;

    /**
     * Initializes the mutex with an initial contained value.
     *
     * @param data The initial value in the mutex.
     */
    explicit Mutex(T value) : mutex_buffer() {
        mutex_handle = xSemaphoreCreateMutexStatic((StaticSemaphore_t*) &mutex_buffer);
        check = mutex_handle;
        data = value;

//        xSemaphoreGive(mutex_handle);
//        memset(mutex_buffer, 0xAD, 32);
//        memset(&mutex_buffer[sizeof(StaticSemaphore_t) + 32], 0xAD, 32);
//         configASSERT(mutex_handle);
    }

    /**
     * @brief Read the current value in the mutex (will spin until it acquires the lock).
     *
     * @return The value in the mutex.
     */
    T read() {
        if (!mutex_handle || mutex_handle != check) {
            Serial.println("Aw shucks");
            Serial.flush();
        }
        xSemaphoreTake(mutex_handle, portMAX_DELAY);
//        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        T ret = data;
        xSemaphoreGive(mutex_handle);
        return ret;
    }

    /**
     * @brief Read the current value in the mutex, will not acquire lock
     *
     * @return The value in the mutex.
     */
    T read_unsync() {
        return data;
    }

    /**
     * @brief reads value in mutex, will acquire lock
     * 
     * @param ptr buffer to store data in
    */
    void read2(T* ptr) {
        if (!mutex_handle || mutex_handle != check) {
            Serial.println("Aw shucks");
            Serial.flush();
        }
        xSemaphoreTake(mutex_handle, portMAX_DELAY);
//        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        *ptr = data;
        xSemaphoreGive(mutex_handle);
    }

    /**
     * @brief Update the value in the mutex (will spin until it acquires the lock).
     *
     * @param value What to update the mutex to.
     */
    void write(T value) {
        if (!mutex_handle || mutex_handle != check) {
            Serial.println("Aw shucks");
            Serial.flush();
        }

        while (!xSemaphoreTake(mutex_handle, MUTEX_TIMEOUT)) { }
        data = value;
        xSemaphoreGive(mutex_handle);

//        if (mutex_buffer[0] != 0xAD) {
//            Serial.println("Overflowing buffer top");
//        }
//
//        if (mutex_buffer[sizeof(StaticSemaphore_t) + 64 - 1] != 0xAD) {
//            Serial.println("Overflowing buffer bottom");
//        }
    }

    // todo lock/unlock?
};

#undef MUTEX_TIMEOUT

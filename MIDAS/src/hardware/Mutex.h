#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define MUTEX_TIMEOUT pdMS_TO_TICKS(100)


/**
 * A mutex which holds a single value.
 *
 * @tparam T The type of the value to hold. Should have copy semantics.
 */
template<typename T>
struct Mutex {
private:
    StaticSemaphore_t mutex_buffer{};
    SemaphoreHandle_t mutex_handle{};

    T data;

public:
    Mutex() = delete;
    Mutex(const Mutex&) = delete;
    Mutex(Mutex&&) = delete;


    /**
     * Initializes the mutex with an initial contained value.
     *
     * @param data The initial value in the mutex.
     */
    explicit Mutex(T data) : mutex_buffer() {
        mutex_handle = xSemaphoreCreateMutexStatic(&mutex_buffer);
        // configASSERT(mutex_handle);
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

#undef MUTEX_TIMEOUT

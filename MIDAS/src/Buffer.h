#pragma once

#include <cstdlib>
#include <array>

/**
 * @class Buffer
 * 
 * @brief Templeted buffer to quickly calculate averages and derivatives for the FSM
 * 
 * @tparam T type of data to hold in the buffer
 * @tparam BUFFER_SIZE size of the buffer
*/
template<typename T, size_t BUFFER_SIZE>
struct Buffer {
public:
    Buffer() = default;

    /**
     * @brief Append data to the end of the circular buffer
     * 
     * @param element Element to add to thebuffer
    */
    void push(T const& element) {
        buffer[tail_idx] = element;
        tail_idx++;
        if (tail_idx == BUFFER_SIZE) {
            tail_idx = 0;
        }
        if (count < BUFFER_SIZE) {
            count++;
        }
        
    }

    /**
     * @brief Reads newest value into the buffer
     * 
     * @param item reference in which to store the most recent reading
     * 
     * @return boolean indicating a successful read
    */
    bool read(T& item) {
        if (count == 0) {
            return false;
        }
        item = buffer[newest_idx()];
        return true;
    }

   /**
     * @brief Reads oldest value into the buffer
     * 
     * @param item reference in which to store the oldest reading
     * 
     * @return boolean indicating a successful read
    */
    bool read_oldest(T&item) {
        if (count == 0) {
            return false;
        }
        item = buffer[oldest_idx()];
        return true;
    }

    template <size_t count>
    std::array<T, count> read_recent() {
        std::array<T, count> arr;

        size_t i = 0;
        size_t idx = newest_idx();
        while (i < count) {
            arr[i++] = buffer[idx++];
            if (idx == BUFFER_SIZE) {
                idx = 0;
            }
        }
        return arr;
    }
    

private:
    /**
     * @brief gets oldest index in the circular buffer
     * 
     * @return the oldest indext in the buffer
    */
    size_t oldest_idx() {
        if (tail_idx < count) {
            return tail_idx + BUFFER_SIZE - count;
        } else {
            return tail_idx - count;
        }
    }

    /**
     * @brief gets newest index in the circular buffer
     * 
     * @return the newest index in the buffer
    */
    size_t newest_idx() {
        if (tail_idx == 0) {
            return BUFFER_SIZE - 1;
        } else {
            return tail_idx - 1;
        }
    }

    size_t tail_idx = 0;
    size_t count    = 0;
    T buffer[BUFFER_SIZE];
};

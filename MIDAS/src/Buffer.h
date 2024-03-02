#pragma once

#include <cstdlib>
#include <array>

template<typename T, size_t BUFFER_SIZE>
struct Buffer {
public:
    Buffer() = default;

    void push(T const& element) {
        buffer[tail_idx++] = element;
        if (tail_idx == BUFFER_SIZE) {
            tail_idx = 0;
        }
        if (count < BUFFER_SIZE) {
            count++;
        }
    }

    bool read(T& item) {
        if (count == 0) {
            return false;
        }
        item = buffer[newest_idx()];
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
    size_t oldest_idx() {
        if (tail_idx < count) {
            return tail_idx + BUFFER_SIZE - count;
        } else {
            return tail_idx - count;
        }
    }

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

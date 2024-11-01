#pragma once

#include <array>

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"
#include "rocket_commands.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

template<typename SensorData>
struct Reading {
    uint32_t timestamp_ms;
    SensorData data;
};

template<typename S>
struct SensorData {
private:
    Mutex<S> current;
    Queue<Reading<S>> queue;

public:
    SensorData() : current(S()) { }

    virtual void update(S data) {
        current.write(data);
        queue.send((Reading<S>) { .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()), .data = data });
    };

    S getRecent() {
        return current.read();
    };

    S getRecentUnsync() {
        return current.read_unsync();
    }

    bool getQueued(Reading<S>* out) {
        return queue.receive(out);
    };
};

template<typename S, size_t count>
struct BufferedSensorData : public SensorData<S> {
private:
    Buffer<S, count> buffer;
    Buffer<TickType_t, count> data_time;

public:
    BufferedSensorData() : SensorData<S>() { }

    void update(S data) override {
        SensorData<S>::update(data);
        buffer.push(data);
        data_time.push(xTaskGetTickCount());
    };

    // wrapper function to get easy access to buffer data
    template<size_t arr_count>
    std::array<S, arr_count> getBufferRecent() {
        std::array<S, arr_count> arr = buffer. template read_recent<arr_count>();
        return arr;
    };

    template<size_t arr_count>
    std::array<TickType_t, arr_count> getTimesRecent() {
        std::array<TickType_t, arr_count> arr = data_time. template read_recent<arr_count>();
        return arr;
    };
};

class Latency {
    uint32_t latency = 0;
    TickType_t last_tick = 0;

public:
    void tick() {
        TickType_t now = xTaskGetTickCount();
        latency = now - last_tick;
        last_tick = now;
    }

    [[nodiscard]] uint32_t getLatency() const {
        return latency;
    }
};

/**
 * The RocketData struct stores all data that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

struct RocketData {
public:
    SensorData<KalmanData> kalman;
    SensorData<LowGData> low_g;
    BufferedSensorData<HighGData, 8> high_g;
    BufferedSensorData<Barometer, 8> barometer;
    SensorData<LowGLSM> low_g_lsm;
    SensorData<Continuity> continuity;
    SensorData<PyroState> pyro;
    SensorData<FSMState> fsm_state;
    SensorData<GPS> gps;
    SensorData<Magnetometer> magnetometer;
    SensorData<Orientation> orientation;
    SensorData<Voltage> voltage;


    Queue<rocketCommands> BluetoothCommands;
    Latency telem_latency;
    Latency log_latency;
};

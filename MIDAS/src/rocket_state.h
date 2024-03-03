#pragma once

#include <array>

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"

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
    void update(S data) {
        current.write(data);
        queue.send((Reading<S>) { .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()), .data = data });
    };

    S getRecent() {
        return current.read();
    };

    bool getQueued(Reading<S>* out) {
        return queue.receive(out);
    };

    SensorData() : current(SensorData()) { }
};

template<typename S, size_t count>
struct BufferedSensorState {
private:
    Mutex<S> current;
    Queue<S> queue;
    Buffer<S, count> buffer;
    Buffer<TickType_t, count> data_time;

public:
    void update(S data) {
        current.write(data);
        queue.send(data);
        buffer.push(data);
        data_time.push(xTaskGetTickCount());
    };

    S getRecent() {
        return current.read();
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

    // function to get the tick when the oldest data point was recorded
    TickType_t getStartTime() {
        TickType_t time;
        if(data_time.read_oldest(time)) {return time;}
        return S();
    }


    bool getQueued(S* out) {
        return queue.receive(out);
    };

    BufferedSensorState() : current(S()) { }
};

enum Stage {
    SUSTAINER,
    BOOSTER,
};

/**
 * The RocketData struct stores all data that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */
struct RocketData {
public:
    bool pyro_should_be_firing = false;
    Stage rocket_stage;

    SensorData<KalmanData> kalman;
    SensorData<LowGData> low_g;
    BufferedSensorState<HighGData, 8> high_g;
    BufferedSensorState<Barometer, 8> barometer;
    SensorData<Continuity> continuity;
    SensorData<Voltage> voltage;
    SensorData<GPS> gps;
    SensorData<Magnetometer> magnetometer;
    SensorData<Orientation> orientation;
    SensorData<FSMState> fsm_state;
};


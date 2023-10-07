#pragma once

#include "sensor_data.h"
#include "hal.h"


template<typename SensorData>
struct Reading {
    uint32_t timestamp_ms;
    SensorData data;
};

template<typename S>
struct SensorData {
public:
    SensorData() : current(S()) { }

    void update(S data) {
        current.write(data);
        queue.send((Reading<S>) { .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()), .data = data });
    };

    S getRecent() {
        return current.read();
    };

    bool getQueued(Reading<S>* out) {
        return queue.receive(out);
    }

private:
    Mutex<S> current;
    Queue<Reading<S>> queue;
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

    SensorData<LowGData> low_g;
    SensorData<HighGData> high_g;
    SensorData<Barometer> barometer;
    SensorData<Continuity> continuity;
    SensorData<Voltage> voltage;
    SensorData<GPS> gps;
    SensorData<Magnetometer> magnetometer;
    SensorData<Orientation> orientation;
};


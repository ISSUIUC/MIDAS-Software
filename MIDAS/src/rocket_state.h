#pragma once

#include "sensor_data.h"
#include "hal.h"

template<typename SensorData>
struct SensorState {
private:
    struct Reading {
        uint32_t timestamp_ms;
        SensorData data;
    };

    Mutex<SensorData> current;
    Queue<Reading> queue;

public:
    SensorState() : current(SensorData()) { }

    void update(SensorData data) {
        current.write(data);
        queue.send((Reading) { .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()), .data = data });
    };

    SensorData getRecent() {
        return current.read();
    };

    bool getQueued(SensorData* out) {
        return queue.receive(out);
    }
};

/**
 * The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */
struct RocketState {
public:
    bool pyro_should_be_firing = false;

    SensorState<LowGData> low_g;
    SensorState<HighGData> high_g;
    SensorState<Barometer> barometer;
    SensorState<Continuity> continuity;
    SensorState<Voltage> voltage;
    SensorState<GPS> gps;
    SensorState<Magnetometer> magnetometer;
    SensorState<Orientation> orientation;
};


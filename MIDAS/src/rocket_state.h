#pragma once

#include "sensor_data.h"
#include "hal.h"

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

    SensorData() : current(S()) { }
};

/**
 * The RocketData struct stores all data that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */
struct RocketData {
public:
    SensorState<LowGData> low_g;
    SensorState<HighGData> high_g;
    SensorState<GyroscopeData> gyroscope;
    SensorState<Barometer> barometer;
    SensorState<Continuity> continuity;
    SensorState<Voltage> voltage;
    SensorState<GPS> gps;
    SensorState<Magnetometer> magnetometer;
    SensorState<Orientation> orientation;
    SensorState<FSMState> fsm_state;
    SensorState<Pyro> pyro;
};


#pragma once

#include "sensor_data.h"
#include "hal.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

template<typename SensorData>
struct SensorState {
private:
    Mutex<SensorData> current;
    Queue<SensorData> queue;

public:
    void update(SensorData data) {
        current.write(data);
        queue.send(data);
    };

    SensorData getRecent() {
        return current.read();
    };

    bool getQueued(SensorData* out) {
        return queue.receive(out);
    };

    SensorState() : current(SensorData()) { }
};

struct RocketState {
public:
    bool pyro_should_be_firing;

    SensorState<LowGData> low_g;
    SensorState<HighGData> high_g;
    SensorState<GyroscopeData> gyroscope;
    SensorState<Barometer> barometer;
    SensorState<Continuity> continuity;
    SensorState<Voltage> voltage;
    SensorState<GPS> gps;
    SensorState<Magnetometer> magnometer;
    SensorState<Orientation> orientation;
};


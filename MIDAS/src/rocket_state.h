#pragma once

#include "sensor_data.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

template<typename SensorData>
struct SensorState {
private:
    Mutex<SensorData> current;
    Queue<Datatype> queue;

public:
    void update(SensorData data);
    SensorData getRecent();
    bool getQueued(SensorData* out);
};

struct RocketState {
    bool pyro_should_be_firing;

    SensorState<LowGData> low_g;
};


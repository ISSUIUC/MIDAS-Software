#pragma once

#include "mcu_main/sensor_data.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

class RocketState {
    bool pyro_should_be_firing;

    Mutex<LowGData> low_g;
    Queue<LowGData> low_g_queue;

public:
    void updateLowG(LowGData data);
    LowGData getRecentLowG();
    bool getQueuedLowG(LowGData* out);
};


#pragma once

#include <array>

#include "finite-state-machines/fsm.h"

#include "sensor_data.h"
#include "hal.h"
#include "Buffer.h"



/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

template<typename S>
struct SensorState {
private:
    Mutex<S> current;
    Queue<S> queue;

public:
    void update(S data) {
        current.write(data);
        queue.send(data);
    };

    S getRecent() {
        return current.read();
    };

    bool getQueued(S* out) {
        return queue.receive(out);
    };

    SensorState() : current(S()) { }
};

template<typename S, size_t count>
struct BufferedSensorState {
private:
    Mutex<S> current;
    Queue<S> queue;
    Buffer<S, count> buffer;

public:
    void update(S data) {
        current.write(data);
        queue.send(data);
        buffer.push(data);
    };

    S getRecent() {
        return current.read();
    };

    // wrapper function to get easy access to buffer data
    std::array<S, count> getBufferRecent() {
        std::array<S, count> arr = buffer. template read_recent<count>(); 
        return arr;
    };
    
    bool getQueued(S* out) {
        return queue.receive(out);
    };

    BufferedSensorState() : current(S()) { }
};

enum stage {
    sustainer = true,
    booster = false
};

struct RocketState {
public:
    bool pyro_should_be_firing;
    bool stage; 
    SensorState<LowGData> low_g;
    BufferedSensorState<HighGData, 8> high_g;
    SensorState<GyroscopeData> gyroscope;
    BufferedSensorState<Barometer, 8> barometer;
    SensorState<Continuity> continuity;
    SensorState<Voltage> voltage;
    SensorState<GPS> gps;
    SensorState<Magnetometer> magnetometer;
    SensorState<Orientation> orientation;
    SensorState<FSMState> fsm_state;
    FSM fsm;
};


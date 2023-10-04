#pragma once

#include "sensor_data.h"
#include "hal.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

enum ReadingDiscriminant {
    READING_low_g,
    READING_high_g,
    READING_barometer,
    READING_continuity,
    READING_voltage,
    READING_gps,
    READING_magnetometer,
    READING_orientation
};

struct Reading {
    ReadingDiscriminant discriminant;
    uint64_t timestamp_ms;
    union {
        LowGData variant_low_g;
        HighGData variant_high_g;
        Barometer variant_barometer;
        Continuity variant_continuity;
        Voltage variant_voltage;
        GPS variant_gps;
        Magnetometer variant_magnetometer;
        Orientation variant_orientation;
    };

    Reading() = delete;
};

#define UPDATE_READING(rocket_state, sensor, value) do { \
                                                        auto _macro_value = value; \
                                                        (rocket_state).sensor._update((_macro_value)); \
                                                        (rocket_state).readings.queue.send(Reading {   \
                                                            .discriminant = ReadingDiscriminant::READING_##sensor, \
                                                            .timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount()),     \
                                                            .variant_##sensor = _macro_value           \
                                                        });                        \
                                                    } while (0)


struct Readings {
    Queue<Reading, 512> queue;
};


template<typename SensorData>
struct SensorState {
private:
    Mutex<SensorData> current;

public:
    void _update(SensorData data) {
        current.write(data);
    };

    SensorData getRecent() {
        return current.read();
    };

    SensorState() : current(SensorData()) { }
};

struct RocketState {
public:
    bool pyro_should_be_firing = false;

    Readings readings;

    SensorState<LowGData> low_g;
    SensorState<HighGData> high_g;
    SensorState<Barometer> barometer;
    SensorState<Continuity> continuity;
    SensorState<Voltage> voltage;
    SensorState<GPS> gps;
    SensorState<Magnetometer> magnetometer;
    SensorState<Orientation> orientation;
};


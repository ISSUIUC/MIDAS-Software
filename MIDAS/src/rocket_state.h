#pragma once

#include "sensor_data.h"
#include "hal.h"

/** The RocketState struct stores everything that is needed by more than one system/thread of the Rocket.
 *
 *  Normally, this would be considered a poor decision. However, the fact that all this data is here
 *  makes it easier to debug since all this data can be logged (and thus used when debugging).
 */

enum ReadingDiscriminant {
    READING_LowGData,
    READING_HighGData,
    READING_Barometer,
    READING_Continuity,
    READING_Voltage,
    READING_GPS,
    READING_Magnetometer,
    READING_Orientation
};

struct Reading {
    ReadingDiscriminant discriminant;
    uint64_t timestamp_ms;
    union {
        LowGData variant_LowGData;
        HighGData variant_HighGData;
        Barometer variant_Barometer;
        Continuity variant_Continuity;
        Voltage variant_Voltage;
        GPS variant_GPS;
        Magnetometer variant_Magnetometer;
        Orientation variant_Orientation;
    };

    Reading() = delete;
};

template<typename SensorData>
struct SensorDataToReading { };

#define ASSOCIATE(ty) template<>                                                        \
                      struct SensorDataToReading<ty> {                                  \
                          static Reading to_reading(ty data, uint64_t timestamp_ms) {   \
                              return Reading {                                          \
                                  .discriminant = ReadingDiscriminant::READING_##ty,    \
                                  .timestamp_ms = timestamp_ms,                         \
                                  .variant_##ty = data                                  \
                              };                                                        \
                          }                                                             \
                      }                                                                 \

ASSOCIATE(LowGData);
ASSOCIATE(HighGData);
ASSOCIATE(Barometer);
ASSOCIATE(Continuity);
ASSOCIATE(Voltage);
ASSOCIATE(GPS);
ASSOCIATE(Magnetometer);
ASSOCIATE(Orientation);

#undef ASSOCIATE

struct Readings {
    Queue<Reading, 512> queue;
};


template<typename SensorData>
struct SensorState {
private:
    Mutex<SensorData> current;
    Readings& readings;

public:
    void update(SensorData data) {
        current.write(data);
        readings.queue.send(SensorDataToReading<SensorData>::to_reading(data, 0));
    };

    SensorData getRecent() {
        return current.read();
    };

//    bool getQueued(SensorData* out) {
//        return queue.receive(out);
//    };

    explicit SensorState(Readings& readings_) : current(SensorData()), readings(readings_) { }
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

    RocketState() : low_g(readings), high_g(readings), barometer(readings), continuity(readings), voltage(readings), gps(readings),
                    magnetometer(readings), orientation(readings) { }
};


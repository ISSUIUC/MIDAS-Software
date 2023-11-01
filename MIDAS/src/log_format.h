#pragma once

#include "sensor_data.h"
#include <tuple>


/**
 * 0 is not used to make it easier to spot bugs
 */
enum ReadingDiscriminant: char {
    ID_LOWG             = 1,
    ID_HIGHG            = 2,
    ID_BAROMETER        = 3,
    ID_CONTINUITY       = 4,
    ID_VOLTAGE          = 5,
    ID_GPS              = 6,
    ID_MAGNETOMETER     = 7,
    ID_ORIENTATION      = 8,
    ID_LOWGLSM          = 9,
    READING_DISCRIMINANT_SIZE
};



template<typename T>
constexpr ReadingDiscriminant get_discriminant();

#define ASSOCIATE(ty, id) template<> constexpr ReadingDiscriminant get_discriminant<ty>() { return ReadingDiscriminant::id; }

ASSOCIATE(LowGData, ID_LOWG)
ASSOCIATE(LowGLSM, ID_LOWGLSM)
ASSOCIATE(HighGData, ID_HIGHG)
ASSOCIATE(Barometer, ID_BAROMETER)
ASSOCIATE(Continuity, ID_CONTINUITY)
ASSOCIATE(Voltage, ID_VOLTAGE)
ASSOCIATE(GPS, ID_GPS)
ASSOCIATE(Magnetometer, ID_MAGNETOMETER)
ASSOCIATE(Orientation, ID_ORIENTATION)

/*
 * This struct isn't actually logged as-is, because if we did we'd waste extra space since
 * unions are the size of their largest member. This is just a reference struct.
 *
 * Instead, we use 4 bytes for the discriminant, 4 bytes for the timestamp, and then write the
 * actual data. No padding between inside these items or between readings.
 * 
 */ 
struct LoggedReading {
    ReadingDiscriminant discriminant;
    uint32_t timestamp_ms;
    union {
        LowGData low_g;
        HighGData high_g;
        Barometer barometer;
        Continuity continuity;
        Voltage voltage;
        GPS gps;
        Magnetometer magnetometer;
        Orientation orientation;
        LowGLSM lowg_lsm;
    } data;
};

using LoggedReadingType = std::tuple<
        LowGData,
        HighGData,
        Barometer,
        Continuity,
        Voltage,
        GPS,
        Magnetometer,
        Orientation,
        LowGLSM
    >;

#pragma once

/**
 * 0 is not used to make it easier to spot bugs
 */
enum ReadingDiscriminant {
    ID_LOWGDATA         = 1,
    ID_HIGHGDATA        = 2,
    ID_BAROMETER        = 3,
    ID_CONTINUITY       = 4,
    ID_VOLTAGE          = 5,
    ID_GPS              = 6,
    ID_MAGNETOMETER     = 7,
    ID_ORIENTATION      = 8,
};


/**
 * This struct isn't actually logged as-is, because if we did we'd waste extra space since
 * unions are the size of their largest member. This is just a reference struct.
 *
 * Instead, we use 4 bytes for the discriminant, 4 bytes for the timestamp, and then write the
 * actual data. No padding between inside these items or between readings.
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
    } data;
};
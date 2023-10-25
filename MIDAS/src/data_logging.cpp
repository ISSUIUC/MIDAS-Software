#include "data_logging.h"
#include "log_format.h"
#include "log_checksum.h"


template<typename T>
constexpr ReadingDiscriminant get_discriminant();

#define ASSOCIATE(ty, id) template<> constexpr ReadingDiscriminant get_discriminant<ty>() { return ReadingDiscriminant::id; }

ASSOCIATE(LowGData, ID_LOWG)
ASSOCIATE(LowGLSM, ID_LOWG_LSM)
ASSOCIATE(HighGData, ID_HIGHG)
ASSOCIATE(Barometer, ID_BAROMETER)
ASSOCIATE(Continuity, ID_CONTINUITY)
ASSOCIATE(Voltage, ID_VOLTAGE)
ASSOCIATE(GPS, ID_GPS)
ASSOCIATE(Magnetometer, ID_MAGNETOMETER)
ASSOCIATE(Orientation, ID_ORIENTATION)


template<typename T>
void log_reading(LogSink& sink, Reading<T>& reading) {
    ReadingDiscriminant discriminant = get_discriminant<T>();
    sink.write((uint8_t*) &discriminant, sizeof(ReadingDiscriminant));
    sink.write((uint8_t*) &reading.timestamp_ms, sizeof(uint32_t));
    sink.write((uint8_t*) &reading.data, sizeof(T));
}

template<typename T>
int log_from_sensor_data(LogSink& sink, SensorData<T>& sensor_data) {
    Reading<T> reading;
    int read = 0;
    while (sensor_data.getQueued(&reading)) {
        log_reading(sink, reading);
        read++;
    }
    return read;
}

void log_begin(LogSink& sink) {
    uint32_t checksum = LOG_CHECKSUM;
    sink.write((uint8_t*) &checksum, 4);
}

void log_data(LogSink& sink, RocketData& data) {
    log_from_sensor_data(sink, data.low_g);
    log_from_sensor_data(sink, data.low_g_lsm);
    log_from_sensor_data(sink, data.high_g);
    log_from_sensor_data(sink, data.barometer);
    log_from_sensor_data(sink, data.continuity);
    log_from_sensor_data(sink, data.voltage);
    log_from_sensor_data(sink, data.gps);
    log_from_sensor_data(sink, data.magnetometer);
    log_from_sensor_data(sink, data.orientation);
}

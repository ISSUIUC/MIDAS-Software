#include "data_logging.h"
#include "log_format.h"
#include "log_checksum.h"

template<typename T>
void log_reading(LogSink& sink, Reading<T>& reading) {
    ReadingDiscriminant discriminant = get_discriminant<T>();
    sink.write((uint8_t*) &discriminant, sizeof(discriminant));
    sink.write((uint8_t*) &reading.timestamp_ms, sizeof(reading.timestamp_ms));
    sink.write((uint8_t*) &reading.data, sizeof(reading.data));
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

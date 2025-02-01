#include "data_logging.h"

#include "log_format.h"
#include "log_checksum.h"

/**
 * @brief Forward decleration of the ID recieving function
*/
template<typename T>
constexpr ReadingDiscriminant get_discriminant();

/**
 * @brief macro to associate a certain sensor with a specific number ID
*/
#define ASSOCIATE(ty, id) template<> constexpr ReadingDiscriminant get_discriminant<ty>() { return ReadingDiscriminant::id; }

ASSOCIATE(LowGData, ID_LOWG)
ASSOCIATE(LowGLSMData, ID_LOWGLSM)
ASSOCIATE(HighGData, ID_HIGHG)
ASSOCIATE(BarometerData, ID_BAROMETER)
ASSOCIATE(ContinuityData, ID_CONTINUITY)
ASSOCIATE(VoltageData, ID_VOLTAGE)
ASSOCIATE(GPSData, ID_GPS)
ASSOCIATE(MagnetometerData, ID_MAGNETOMETER)
ASSOCIATE(OrientationData, ID_ORIENTATION)
ASSOCIATE(FSMState, ID_FSM)
ASSOCIATE(KalmanData, ID_KALMAN)
ASSOCIATE(PyroState, ID_PYRO)

/**
 * @brief writes a reading, with its ID, timestamp, and data to a specific sink
 * 
 * @param sink the LogSink to write to
 * @param reading the data to read
*/
template<typename T>
void log_reading(ILogSink& sink, Reading<T>& reading) {
    ReadingDiscriminant discriminant = get_discriminant<T>();
    sink.write((uint8_t*) &discriminant, sizeof(ReadingDiscriminant));
    sink.write((uint8_t*) &reading.timestamp_ms, sizeof(uint32_t));
    sink.write((uint8_t*) &reading.data, sizeof(T));
}

/**
 * @brief writes a SensorData's entire queue reading to a sink
 * 
 * @param sink the LogSink to write to
 * @param sensor_data the sensor data, with queue, to write from
 * 
 * @return the number of packets written to the LogSink
*/
template<typename T>
uint32_t log_from_sensor_data(ILogSink& sink, SensorData<T>& sensor_data) {
    Reading<T> reading = {};
    uint32_t read = 0;
    while (read < 20 && sensor_data.getQueued(&reading)) {
        log_reading(sink, reading);
        read++;
    }
    return read;
}

/**
 * @brief Initializes a specific LogSink
 * 
 * @param sink the LogSink to initialize
*/
void log_begin(ILogSink& sink) {
    uint32_t checksum = LOG_CHECKSUM;
    sink.write((uint8_t*) &checksum, 4);
}

/**
 * @brief logs all sensor data from the rocket
 * 
 * @param sink the LogSink to write data to
 * @param data the rocket which holds all the sensor data to write
*/
void log_data(ILogSink& sink, RocketData& data) {
    log_from_sensor_data(sink, data.low_g);
    log_from_sensor_data(sink, data.low_g_lsm);
    log_from_sensor_data(sink, data.high_g);
    log_from_sensor_data(sink, data.barometer);
    log_from_sensor_data(sink, data.continuity);
    log_from_sensor_data(sink, data.voltage);
    log_from_sensor_data(sink, data.gps);
    log_from_sensor_data(sink, data.magnetometer);
    log_from_sensor_data(sink, data.orientation);
    log_from_sensor_data(sink, data.fsm_state);
    log_from_sensor_data(sink, data.kalman);
    log_from_sensor_data(sink, data.pyro);
}

#ifndef SILSIM
#endif

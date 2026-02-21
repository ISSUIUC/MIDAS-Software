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


ASSOCIATE(IMU, ID_IMU)
ASSOCIATE(Barometer, ID_BAROMETER)
ASSOCIATE(Voltage, ID_VOLTAGE)
ASSOCIATE(GPS, ID_GPS)
ASSOCIATE(Magnetometer, ID_MAGNETOMETER)
ASSOCIATE(KalmanData, ID_KALMAN)
ASSOCIATE(FSMState, ID_FSM)
ASSOCIATE(PyroState, ID_PYRO)
ASSOCIATE(CameraData, ID_CAMERADATA)
ASSOCIATE(AngularKalmanData, ID_ANGULARKALMAN)
ASSOCIATE(IMU_SFLP, ID_SFLPHW)


/**
 * @brief writes a reading, with its ID, timestamp, and data to a specific sink
 * 
 * @param sink the LogSink to write to
 * @param reading the data to read
*/
template<typename T>
void log_reading(LogSink& sink, Reading<T>& reading) {
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
uint32_t log_from_sensor_data(LogSink& sink, SensorData<T>& sensor_data) {
    Reading<T> reading;
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
void log_begin(LogSink& sink) {
    uint32_t checksum = LOG_CHECKSUM;
    sink.write((uint8_t*) &checksum, 4);
}

/**
 * @brief logs all sensor data from the rocket
 * 
 * @param sink the LogSink to write data to
 * @param data the rocket which holds all the sensor data to write
*/
void log_data(LogSink& sink, RocketData& data) {
    log_from_sensor_data(sink, data.imu);
    log_from_sensor_data(sink, data.hw_filtered);
    log_from_sensor_data(sink, data.barometer);
    log_from_sensor_data(sink, data.voltage);
    log_from_sensor_data(sink, data.gps);
    log_from_sensor_data(sink, data.magnetometer);
    //log_from_sensor_data(sink, data.orientation);
    log_from_sensor_data(sink, data.fsm_state);
    log_from_sensor_data(sink, data.kalman);
    log_from_sensor_data(sink, data.angular_kalman_data);
    log_from_sensor_data(sink, data.pyro);

    //log_from_sensor_data(sink, data.quaternions);
    log_from_sensor_data(sink, data.cam_data);

}

#ifndef SILSIM
#define MAX_FILES 999

/**
 * @brief names a new file for a log sink depending on the files currently on said LogSink
 * 
 * @param fileName buffer to write the file name to
 * @param fileExtensionParam the file extension required for the file
 * @param fs the FileSystem to check files off of
 * 
 * @return buffer contianing string of file name
*/
char* sdFileNamer(char* fileName, char* fileExtensionParam, FS& fs, uint16_t file_num, int* fileno_out) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[256] = {0};
    strcpy(inputName, "/");
    strcat(inputName, fileName);
    strcat(inputName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    bool exists = fs.exists(inputName);

    if (exists) {
        bool fileExists = false;
        // We will start at file_num, which is default 0 if eeprom is erased
        int i = file_num;
        while (!fileExists) {
            if (i > MAX_FILES) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(inputName, "/");
                strcat(inputName, fileName);
                strcat(inputName, "999");
                strcat(inputName, fileExtension);
                *fileno_out = 999;
                break;
            }

            // converts int i to char[]
            char iStr[16] = {0};
            itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            strcpy(inputName, "/");
            strcat(inputName, fileName);
            strcat(inputName, iStr);
            strcat(inputName, fileExtension);

            if (!fs.exists(inputName)) {
                fileExists = true;
                *fileno_out = i;
            }

            i++;
        }
    }

    strcpy(fileName, inputName);

    return fileName;
}
#endif
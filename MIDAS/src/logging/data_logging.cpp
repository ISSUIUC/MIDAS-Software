#include "data_logging.h"
#include "log_format.h"
#include "log_checksum.h"

/**
 * @brief Forward declaration for retrieving the log discriminant associated
 *        with a specific reading type.
 *
 * Each supported sensor or subsystem type has a unique ReadingDiscriminant
 * value used to identify serialized log packets during parsing.
 */
template<typename T>
constexpr ReadingDiscriminant get_discriminant();

/**
 * @brief Writes a single sensor reading to the specified logging sink.
 *
 * Each logged reading is serialized in the following order:
 * 1. Reading type identifier (ReadingDiscriminant)
 * 2. Timestamp in milliseconds
 * 3. Raw reading data
 *
 * This standardized format allows log parsers to identify packet types while
 * replaying or analyzing flight logs.
 *
 * @tparam T Sensor data type contained in the reading.
 * @param sink Destination logging backend.
 * @param reading Timestamped sensor reading to serialize.
 */
template<typename T>
void log_reading(LogSink& sink, Reading<T>& reading) {
    ReadingDiscriminant discriminant = get_discriminant<T>();
    sink.write((uint8_t*) &discriminant, sizeof(ReadingDiscriminant));
    sink.write((uint8_t*) &reading.timestamp_ms, sizeof(uint32_t));
    sink.write((uint8_t*) &reading.data, sizeof(T));
}

/**
 * @brief Flushes queued sensor readings to the logging sink.
 *
 * Reads queued samples from a SensorData object and writes each one using
 * log_reading(). To prevent logging from monopolizing execution time, the
 * function writes at most 20 readings per invocation.
 *
 * @tparam T Sensor data type stored by the SensorData queue.
 * @param sink Destination logging backend.
 * @param sensor_data Sensor queue containing pending readings.
 *
 * @return Number of readings successfully written.
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
 * @brief Begins a new flight log.
 *
 * Writes a fixed checksum header to the beginning of the log file so that
 * parsing tools can verify the file format before processing logged data.
 *
 * @param sink Logging backend to initialize.
 */
void log_begin(LogSink& sink) {
    uint32_t checksum = LOG_CHECKSUM;
    sink.write((uint8_t*) &checksum, 4);
}

/**
 * @brief Logs all available queued flight data.
 *
 * Flushes pending readings from every major subsystem queue, including sensor
 * measurements, state estimation, FSM state, pyro status, and camera data.
 *
 * @param sink Destination logging backend.
 * @param data Rocket data structure containing all logging queues.
 */
void log_data(LogSink& sink, RocketData& data) {
    log_from_sensor_data(sink, data.imu);
    log_from_sensor_data(sink, data.sflp);
    log_from_sensor_data(sink, data.barometer);
    log_from_sensor_data(sink, data.voltage);
    log_from_sensor_data(sink, data.gps);
    log_from_sensor_data(sink, data.magnetometer);
    log_from_sensor_data(sink, data.fsm_state);
    log_from_sensor_data(sink, data.kalman);
    log_from_sensor_data(sink, data.angular_kalman_data);
    log_from_sensor_data(sink, data.pyro);
    log_from_sensor_data(sink, data.cam_data);
}

#ifndef SILSIM
#define MAX_FILES 999

/**
 * @brief Generates a unique filename for a new flight log.
 *
 * Searches the filesystem for existing files using the supplied base name and
 * extension. If a matching filename already exists, an incrementing numeric
 * suffix is appended until an unused filename is found or the maximum file
 * number is reached.
 *
 * The selected filename is written back into the supplied buffer.
 *
 * @param fileName Buffer containing the base filename. On return, contains the
 *        generated filename including path and numeric suffix.
 * @param fileExtensionParam Desired file extension (e.g. ".bin").
 * @param fs Filesystem used to check for existing files.
 * @param file_num Starting file number, typically recovered from EEPROM.
 * @param fileno_out Pointer that receives the next file number.
 *
 * @return Pointer to the updated filename buffer.
 */
char* sdFileNamer(char* fileName, char* fileExtensionParam, FS& fs, uint16_t file_num, int* fileno_out) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[256] = {0};
    strcpy(inputName, "/");
    strcat(inputName, fileName);
    strcat(inputName, fileExtension);

    // Check whether the base filename already exists.
    bool exists = fs.exists(inputName);

    if (exists) {
        bool fileExists = false;

        // Start searching from the recovered file number.
        int i = file_num;

        while (!fileExists) {
            if (i > MAX_FILES) {
                // Maximum file number reached. Reuse the final filename to avoid
                // overflowing the filename buffer.
                strcpy(inputName, "/");
                strcat(inputName, fileName);
                strcat(inputName, "999");
                strcat(inputName, fileExtension);
                *fileno_out = 999;
                break;
            }

            // Convert the current file number into a string.
            char iStr[16] = {0};
            itoa(i, iStr, 10);

            // Construct "/data<number>.ext".
            strcpy(inputName, "/");
            strcat(inputName, fileName);
            strcat(inputName, iStr);
            strcat(inputName, fileExtension);

            // Stop once an unused filename is found.
            if (!fs.exists(inputName)) {
                fileExists = true;
                *fileno_out = i + 1;
            }

            i++;
        }
    } else {
        // Base filename is unused.
        *fileno_out = 0;
    }

    strcpy(fileName, inputName);

    return fileName;
}
#endif
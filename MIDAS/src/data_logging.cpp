#include "data_logging.h"
#include "log_format.h"
#include "log_checksum.h"

#define MAX_FILES 999

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


template<typename T>
void log_reading(Logger& sink, Reading<T>& reading) {
    ReadingDiscriminant discriminant = get_discriminant<T>();
    sink.write((uint8_t*) &discriminant, sizeof(ReadingDiscriminant));
    sink.write((uint8_t*) &reading.timestamp_ms, sizeof(uint32_t));
    sink.write((uint8_t*) &reading.data, sizeof(T));
}

template<typename T>
int log_from_sensor_data(Logger& sink, SensorData<T>& sensor_data) {
    Reading<T> reading;
    int read = 0;
    while (sensor_data.getQueued(&reading)) {
        log_reading(sink, reading);
        read++;
    }
    return read;
}

void log_begin(Logger& sink) {
    uint32_t checksum = LOG_CHECKSUM;
    sink.write((uint8_t*) &checksum, 4);
}

void log_data(Logger& sink, RocketData& data) {
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

char* sdFileNamer(char* fileName, char* fileExtensionParam, int select) {
    char fileExtension[strlen(fileExtensionParam) + 1];
    strcpy(fileExtension, fileExtensionParam);

    char inputName[strlen(fileName) + 1];
    strcpy(inputName, fileName);

    strcat(fileName, fileExtension);

    // checks to see if file already exists and adds 1 to filename if it does.
    bool SD_Exists = (select == 0) && SD.exists(fileName);
    bool EMMC_Exists = (select == 1) && SD_MMC.exists(fileName);

    bool exists = SD_Exists || EMMC_Exists;

    if (exists) {
        bool fileExists = false;
        int i = 1;
        while (!fileExists) {
            if (i > MAX_FILES) {
                // max number of files reached. Don't want to overflow
                // fileName[]. Will write new data to already existing
                // data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            // converts int i to char[]
            char iStr[16];
            itoa(i, iStr, 10);

            // writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName) + strlen(iStr) + 6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp, iStr);
            strcat(fileNameTemp, fileExtension);

            if (!SD.exists(fileNameTemp)) {
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    return fileName;
}
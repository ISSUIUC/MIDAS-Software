#include "simulated_sensors.h"

using namespace std;



// ErrorCode IMUCalibrationState
struct Sensor_File {
    std::ifstream stream;
    std::vector<std::string> headers;
};

std::unordered_map<std::string, Sensor_File> sensor_file_objects;

bool valid_sensor_name(std::string sensor_name) {
    // check if it's valid
    return sensor_file_objects.contains(sensor_name);
}

void silsim_init(std::string sensor_name) {
    if (!valid_sensor_name(sensor_name))
        throw std::runtime_error("invalid sensor name in init"); 
    std::ifstream sensor_file(std::filesystem::current_path() + "/data/sensors/" + sensor_name + ".csv");
    if (!sensor_file.is_open()) {
        std::cerr << "file did not open well" << std::endl;
    }
    sensor_file_objects[sensor_name] = Sensor_File {
        .stream = sensor_file,
        .headers = 
    }
}

std::vector<double> silsim_read(std::string sensor_name) {
    // assume sensor_name is appropriate to pull from an existing csv
    // get current time_stamp
    // use csv from corresponding sensor
    // find the row for the corresponding time_stamp
    // put the row in a vector<double>
    // return the vector
    if (!valid_sensor_name(sensor_name)) throw std::runtime_error("invalid sensor name in read"); 
    std::ifstream sensor_file = sensor_file_objects[sensor_name];

    int timestamp = pdTICKS_TO_MS(xTaskGetTickCount()); // get the current time
    // maybe wrong? Probably okay

    std::stringstream ss;
    if (!sensor_file.is_open()) {
        throw std::runtime_error("sensor file not open");
    }

    std::string line;
    std::streampos start_of_last_line = sensor_file.tellg();

    std::vector<double> out;

    while (std::getline(sensor_file, line)) {
        std::string csv_timestamp;
        ss.string(""); // reset ss
        ss << line; // read line into ss
        // read and parse and do stuff
        // check the time and whatnot
        std::getline(ss, csv_timestamp, ',');
        if (std::stoi(csv_timestamp) <= timestamp) {
            start_of_last_line = sensor_file.tellg();
            continue;
        } else {
            sensor_file.seekg(start_of_last_line);
            break;
        }
    }

    std::string csv_value;
    std::getline(sensor_file, line);
    ss.string("");
    ss << line;
    sensor_file.seekg(start_of_last_line); // reset sensor_file
    
    // put everything into return vector
    while(std::getline(ss, csv_value, ',')) {
        out.push_back(std::stod(csv_value));
    }

    return out;
}

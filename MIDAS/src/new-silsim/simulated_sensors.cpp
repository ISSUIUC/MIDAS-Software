#include "simulated_sensors.h"

using namespace std;



// ErrorCode IMUCalibrationState

std::unordered_map<std::string, std::ifstream> sensor_file_objects;

bool valid_sensor_name(std::string sensor_name) {
    // check if its valid

    return true; // validity
}

void silsim_init(std::string sensor_name) {
    if (!valid_sensor_name(sensor_name)) throw std::runtime_error("invalid sensor name in init"); 
    std::ifstream sensor_file(std::filesystem::current_path() + "/data/sensors/" + sensor_name + ".csv");
    if (!sensor_file.is_open()) {
        std::cerr << "file did not open well" << std::endl;
        
    }
    sensor_file_objects[sensor_name] = sensor_file;
}


std::vector<double>* silsim_read(std::string sensor_name){
    // assume sensor_name is appropriate to pull from an existing csv
    // get current time_stamp
    // use csv from corresponding sensor
    // find the row for the corresponding time_stamp
    // put the row in a vector<double>
    // return the vector
    if (!valid_sensor_name(sensor_name)) throw std::runtime_error("invalid sensor name in read"); 
    std::ifstream sensor_file = sensor_file_objects[sensor_name];

    int timestamp = pdTICKS_TO_MS(xTaskGetTickCount()); // get the current time

    std::stringstream ss;
    if ()
    std::string line;
    while (std::getline(sensor_file, line)) {
        std::string csv_timestamp;
        // read and parse and do stuff
        // check the time and whatnot
        std::getline(ss, csv_timestamp, ',');
        if (std::stoi(csv_timestamp) <= timestamp) {
            // do something
            continue;
        }else{
            break;
        }
    }
    std::vector<double> out;

    std::string csv_value
    while(std::getline(ss, csv_value, ',')){
        out->push_back((double)csv_value);
    }

    return &out;
}

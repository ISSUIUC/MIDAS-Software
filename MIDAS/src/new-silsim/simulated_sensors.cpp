#include "simulated_sensors.h"
#include <sstream>
#include <utility>

using namespace std;
// TODO DESTRUCTOR FOR sensor_file_objects unordered map elements that are on heap



// ErrorCode IMUCalibrationState
struct Sensor_File {
    ifstream stream;
    vector<string> headers;
    map<string, double> current_values;
};

unordered_map<string, Sensor_File> sensor_file_objects;

bool valid_sensor_name(string sensor_name) {
    // check if it's valid
    return sensor_file_objects.contains(sensor_name);
}


// buncha stuff i think is not in right scope and wil die when leaving scope so needs to be in global scope 
// //or on heap or something
// jsut watch out for stack stuff
void silsim_init(string sensor_name) {
    if (sensor_name != "imu" && sensor_name != "sflp")
        throw runtime_error("cannot support sensor: " + sensor_name); 
    
    
    // std::unique_ptr<Sensor_File> sensor_file_unique_ptr = std::make_unique<Sensor_File>();
    // sensor_file_unique_ptr->stream.open((filesystem::current_path() + "/data/sensors/" + sensor_name + ".csv"))

    // ifstream sensor_file((filesystem::current_path() + "/data/sensors/" + sensor_name + ".csv"));
    ifstream sensor_file(filesystem::current_path() / "data" / "sensors" / (sensor_name + ".csv"));
    // if (!sensor_file_unique_ptr->stream.is_open()) {
    //     throw runtime_error("could not open sensor CSV: " + sensor_name);
    //     // cerr << "file did not open well" << endl;
    // }

    

    if (!sensor_file.is_open()) {
        throw runtime_error("could not open sensor CSV: " + sensor_name);
    }
    
    string line;
    if (!getline(sensor_file, line)) {
        throw runtime_error("missing csv header: " + sensor_name);
    }

    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    vector<string> all_headers;
    stringstream ss(line);
    string csv_header;

    while (getline(ss, csv_header, ',')) {
        all_headers.push_back(csv_header);
    }


    sensor_file_objects[sensor_name] = Sensor_File{
        .stream = std::move(sensor_file),
        .headers = std::move(all_headers),
        .current_values = {}
    };
}

//needa make it map correctly
map<string, double> silsim_read(string sensor_name, uint64_t timestamp_ms) {
    // assume sensor_name is appropriate to pull from an existing csv
    // get current time_stamp
    // use csv from corresponding sensor
    // find the row for the corresponding time_stamp
    // put the row in a vector<double>
    // return the vector
    if (!valid_sensor_name(sensor_name)) throw runtime_error("invalid sensor name in read"); 
    ifstream& sensor_file = sensor_file_objects.at(sensor_name).stream;

    // int timestamp = pdTICKS_TO_MS(xTaskGetTickCount()); // get the current time
    // maybe wrong? Probably okay

    // stringstream ss;
    if (!sensor_file.is_open()) {
        throw runtime_error("sensor file not open");
    }

    string line;
    // streampos start_of_last_line = sensor_file.tellg();

    // map<string, double> out;

    // problem here is where we save the stream position. need to save the eligible row's values while remembering the position before each row so we can rewind when encountering future data
/*    while (getline(sensor_file, line)) {
        string csv_timestamp;
        ss.string(""); // reset ss
        ss << line; // read line into ss
        // read and parse and do stuff
        // check the time and whatnot
        getline(ss, csv_timestamp, ',');
        if (stoi(csv_timestamp) <= timestamp) {
            start_of_last_line = sensor_file.tellg();
            continue;
        } else {
            sensor_file.seekg(start_of_last_line);
            break;
        }
    }
*/


    auto& sensor = sensor_file_objects.at(sensor_name);
    const auto& headers = sensor.headers;
    while (true) {
        const streampos rowStart = sensor_file.tellg();

        if (!getline(sensor_file, line)) {
            break;
        }

        stringstream rowStream(line);
        string csvTimestamp;
        getline(rowStream, csvTimestamp, ',');

        if (stoull(csvTimestamp) > timestamp_ms) {
            sensor_file.seekg(rowStart);
            break;
        }

        map<string, double> parsedValues;
        string csvValue;

        for (size_t i = 1; i < headers.size(); i++) {
            if (!getline(rowStream, csvValue, ',')) {
                throw runtime_error("missing csv field: " + headers[i]);
            }

            parsedValues[headers[i]] = stod(csvValue);
        }

        sensor.current_values = std::move(parsedValues);

        // now pasre this eligible row into the stored current_values here
    }

    if (sensor.current_values.empty()) {
        throw runtime_error("no sample available");
    }

    return sensor.current_values;

    // this reads another row after selecting by timestamp which could return a future sample. also its local out disappears after each call. the stored current_values saves the last eligible reading when the next call finds no new sample

    // vector<string> headers = sensor_file_objects.at(sensor_name).headers;

    // string csv_value;
    // getline(sensor_file, line);
    // ss.string("");
    // ss << line;
    // sensor_file.seekg(start_of_last_line); // reset sensor_file
    
    // // put everything into return vector
    // int header_index = 0;
    // while(getline(ss, csv_value, ',')) {
    //     // out.push_back(stod(csv_value));
    //     out[headers[header_index]] = stod(csv_value);
    //     header_index++;
    // }

    // return out;
}

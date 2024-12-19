#include <iostream>
#include <fstream>
#include <sstream>  
#include "sensor_data.h"

void writeStructSizesToJson() {
    // Create an ostringstream to build the JSON string
    std::ostringstream jsonStream;

    // Start the JSON object
    jsonStream << "{\n";

    // Add struct sizes as key-value pairs
    /*jsonStream << "  \"Vec3\": " << sizeof(Vec3) << ",\n";
    jsonStream << "  \"Position\": " << sizeof(Position) << ",\n";
    jsonStream << "  \"Velocity\": " << sizeof(Velocity) << ",\n";
    jsonStream << "  \"Acceleration\": " << sizeof(Acceleration) << ",\n";
    jsonStream << "  \"euler_t\": " << sizeof(euler_t) << ",\n";*/
    jsonStream << "  \"1\": " << sizeof(LowGData) << ",\n";
    jsonStream << "  \"2\": " << sizeof(HighGData) << ",\n";
    jsonStream << "  \"9\": " << sizeof(LowGLSM) << ",\n";
    jsonStream << "  \"3\": " << sizeof(Barometer) << ",\n";
    jsonStream << "  \"4\": " << sizeof(Continuity) << ",\n";
    jsonStream << "  \"5\": " << sizeof(Voltage) << ",\n";
    jsonStream << "  \"6\": " << sizeof(GPS) << ",\n";
    jsonStream << "  \"7\": " << sizeof(Magnetometer) << ",\n";
    jsonStream << "  \"8\": " << sizeof(Orientation) << ",\n";
    jsonStream << "  \"11\": " << sizeof(KalmanData) << ",\n";
    //jsonStream << "  \"PyroChannel\": " << sizeof(PyroChannel) << ",\n";
    jsonStream << "  \"12\": " << sizeof(PyroState) << "\n";

    // End the JSON object
    jsonStream << "}";

    // Convert the stream to a string
    std::string jsonString = jsonStream.str();

    // Write the JSON string to a file
    std::ofstream file("struct_sizes.json");
    if (file.is_open()) {
        file << jsonString;  // Write the JSON string to the file
        file.close();
        std::cout << "Struct sizes written to struct_sizes.json\n";
    } else {
        std::cerr << "Error: Could not open file for writing.\n";
    }
}

int main() {
    writeStructSizesToJson();
    return 0;
}

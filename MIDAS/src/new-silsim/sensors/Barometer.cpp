#include "simulated_sensors.h"


ErrorCode BarometerSensor::init() {
    silsim_init("barometer");

    return ErrorCode::NoError;
}

Barometer BarometerSensor::read() {
	std::map<std::string, double> data = silsim_read("barometer");
	return Barometer(data["temperature"], data["pressure"], data["altitude"]);
}

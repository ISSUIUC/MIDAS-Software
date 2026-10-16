#include "simulated_sensors.h"
#include <ads7138-q1.h>

/**
 * @brief "Initializes" the voltage sensor. Since it reads directly from a pin without a library, there is no specific initialization.
 * 
 * @return Error Code, will always be NoError
*/
ErrorCode VoltageSensor::init() {
    silsim_init("voltage");
    return ErrorCode::NoError;
}

/**
 * @brief Reads the value of the given analog pin and converts it to a battery voltage with the assumption that the voltage sensor is plugged into that pin
 * 
 * @return The scaled voltage given by the voltage sensor
*/
Voltage VoltageSensor::read() {
    std::map<double> data = silsim_read("voltage");
    Voltage voltage;
    voltage.continuity[0] = data["continuity[0]"];
    voltage.continuity[1] = data["continuity[1]"];
    voltage.continuity[2] = data["continuity[2]"];
    voltage.continuity[3] = data["continuity[3]"];
    voltage.v_Bat = data["v_bat"];
    voltage.v_Pyro = data["v_Pyro"];
    return voltage;
}

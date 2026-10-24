#include "simulated_sensors.h"
#include <exception>
#include <iostream>



int main() {
    try {
        IMUSensor imu;

        const ErrorCode result = imu.init();

        if (result != ErrorCode::NoError) {
            std::cerr << "failed imu initialition\n";
            return 1;
        }

        const IMU reading = imu.read();
    std::cout << "high-g: " << reading.highg_acceleration.ax << ", " << reading.highg_acceleration.ay << ", " << reading.highg_acceleration.az << '\n';

    }  catch (const std::exception& error) {
        std::cerr << "failed: " << error.what() << '\n';
        return 1;
    }
}

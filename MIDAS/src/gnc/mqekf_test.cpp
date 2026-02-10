#include "mqekf.h"
#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <fstream>
using namespace Eigen;

int main(int argc, char *argv[])

{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_csv_file>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    Eigen::Matrix<float, 3, 1> sigma_a = {160 * sqrt(100.0f) * 1e-6 * 9.81, 160 * sqrt(100.0f) * 1e-6 * 9.81, 190 * sqrt(100.0f) * 1e-6 * 9.81}; // ug/sqrt(Hz) *sqrt(hz). values are from datasheet
    Eigen::Matrix<float, 3, 1> sigma_g = {0.1 * M_PI / 180, 0.1 * M_PI / 180, 0.1 * M_PI / 180};                                                 // 0.1 deg/s
    Eigen::Matrix<float, 3, 1> sigma_m = {0.4e-4 / sqrt(3), 0.4e-4 / sqrt(3), 0.4e-4 / sqrt(3)};                                                 // 0.4 mG -> T, it is 0.4 total so we divide by sqrt3

    // QuaternionMEKF mekf(sigma_a, sigma_g, sigma_m);

    // Eigen::Matrix<float, 3, 1> acc0 = {9.81, 0, 0};         // Factored in
    // Eigen::Matrix<float, 3, 1> mag0 = {-0.34, -0.01, 0.75}; // Factored in ??

    // mekf.initialize_from_acc_mag(acc0, mag0);
    Eigen::Matrix<float, 4, 1> quat = mekf.quaternion();

    std::cout << "[" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]," << std::endl;

    Eigen::Matrix<float, 3, 1> gyr;
    Eigen::Matrix<float, 3, 1> acc;
    Eigen::Matrix<float, 3, 1> mag;
    Eigen::Matrix<float, 2, 1> gps;
    Eigen::Matrix<float, 3, 1> orientation;
    double alt;
    double time;

    std::ifstream file(input_file);

    if (!file.is_open())
    {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    // Vectors to store the sensor data. Each element is a 3D vector
    std::vector<std::vector<double>> accel_pull;    // [ax, ay, az] for each timestamp
    std::vector<std::vector<double>> gyro_pull;     // [wx, wy, wz] for each timestamp rad/s
    std::vector<std::vector<double>> mag_pull;      // [mx, my, mz] for each timestamp
    std::vector<std::vector<double>> gps_pull;      // [lat,long] for each timestamp
    std::vector<std::vector<double>> altitude_pull; // [alt] for each timestamp
    std::vector<std::vector<double>> time_pull;     // [time] for each timestamp

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(ss, cell, ','))
        {
            row.push_back(cell);
        }

        // Extract the sensor values (columns 16-24)
        if (row.size() > 24)
        {
            // std::vector<double> accel_sample = {std::stod(row[16]), std::stod(row[17]), std::stod(row[18])};
            // std::vector<double> gyro_sample = {std::stod(row[19]), std::stod(row[20]), std::stod(row[21])};
            // std::vector<double> mag_sample = {std::stod(row[22]), std::stod(row[23]), std::stod(row[24])};

            std::vector<double> accel_sample = {std::stod(row[16]), std::stod(row[17]), std::stod(row[18])};
            std::vector<double> gyro_sample = {std::stod(row[19]), std::stod(row[20]), std::stod(row[21])};
            std::vector<double> mag_sample = {std::stod(row[24]), std::stod(row[25]), std::stod(row[26])};
            std::vector<double> gps_sample = {std::stod(row[34]), std::stod(row[35])};
            std::vector<double> altitude_sample = {std::stod(row[9])};
            std::vector<double> time_sample = {std::stod(row[4])};
            

            // std::vector<double> accel_scaled = {
            //     accel_sample[0] * 9.81,
            //     accel_sample[1] * 9.81,
            //     accel_sample[2] * 9.81};
            std::vector<double> accel_scaled = {
                accel_sample[0] *1,
                accel_sample[1] *1 ,
                accel_sample[2] *-1};
            accel_pull.push_back(accel_scaled);
            gyro_pull.push_back(gyro_sample);
            mag_pull.push_back(mag_sample);
            gps_pull.push_back(gps_sample);
            altitude_pull.push_back(altitude_sample);
            time_pull.push_back(time_sample);
        }
    }
    file.close();

    std::ofstream outfile("../output/mqekf_quaternion_output.csv", std::ios::out | std::ios::trunc);

    if (!outfile.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }
    // outfile << quat[0] << "," << quat[1] << "," << quat[2] << "," << quat[3] << "\n";
    if (accel_pull.size() != gyro_pull.size() || accel_pull.size() != mag_pull.size())
    {
        std::cerr << "Sensor data size mismatch!" << std::endl;
        return 1;
    }

  

    quat = mekf.quaternion();

    Eigen::Matrix<float, 3, 1> bias = mekf.gyroscope_bias();

    outfile.close();
}

#ifndef COORDS_HPP
#define COORDS_HPP

#include "coords.hpp"
#include <cmath>
#include <vector>

double norm_pi(double angle) {
    return (angle + M_PI) % (2 * M_PI) - M_PI;
}

std::vector<double> matrixMult(const std::vector<std::vector<double> >& R, const std::vector<double>& ECEF_relative) {
    std::vector<double> result(3, 0.0);
    
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            result[i] += R[i][j] * ECEF_relative[j];
        }
    }
    
    return result;
}

std::vector<double> GPS_to_ECEF(std::vector<double> GPS) {
    GPS[0] *= M_PI / 180;
    GPS[1] *= M_PI / 180;
    
    double e2 = (std::pow(a, 2) - std::pow(b, 2)) / std::pow(a, 2);
    double N = a / std::pow(1 - e2 * sin(GPS[0]), .5);

    double x = (N + GPS[2]) * cos(GPS[0]) * cos(GPS[1]);
    double y = (N + GPS[2]) * cos(GPS[0]) * cos(GPS[1]);
    double z = ((1 - e2) * N + GPS[2]) * sin(GPS[0]);

    std::vector<double> ECEF = {x, y, z};
    return ECEF;
}

std::vector<double> ECEF_to_ENU(std::vector<double> pos_turr_GPS, std::vector<double> pos_turr_ECEF, std::vector<double> rocket_ECEF) {
    pos_turr_GPS[0] *= M_PI / 180;
    pos_turr_GPS[1] *= M_PI / 180;
        
    std::vector<double> e = {-sin(pos_turr_GPS[1]), cos(pos_turr_GPS[1]), 0};
    std::vector<double> n = {-sin(pos_turr_GPS[0]) * cos(pos_turr_GPS[1]), -sin(pos_turr_GPS[0]) * sin(pos_turr_GPS[1]), cos(pos_turr_GPS[0])};
    std::vector<double> u = {cos(pos_turr_GPS[0]) * cos(pos_turr_GPS[1]), cos(pos_turr_GPS[0]) * sin(pos_turr_GPS[1]), sin(pos_turr_GPS[0])};

    std::vector<std::vector<double> > R = {{e[0], e[1], e[2]}, {n[0], n[1], n[2]}, {u[0], u[1], u[2]}};
    
    std::vector<double> ECEF_relative = {3, 0.0};
    for (int i = 0; i < 3; ++i){
        ECEF_relative[i] = rocket_ECEF[i] - pos_turr_ECEF[i];
    }

    std::vector<double> ENU = matrixMult(R, ECEF_relative);
    return ENU;
}
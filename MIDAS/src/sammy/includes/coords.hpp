#ifndef COORDS_HPP
#define COORDS_HPP

#include <cmath>
#include <vector>

double a = 6378137.0;
double b = 6356752.3142;

double norm_pi(double angle) {}
std::vector<double> matrixMult(const std::vector<std::vector<double> >& R, const std::vector<double>& ECEF_relative) {}
std::vector<double> GPS_to_ECEF(std::vector<double> GPS) {}
std::vector<double> ECEF_to_ENU(std::vector<double> pos_turr_GPS, std::vector<double> pos_turr_ECEF, std::vector<double> rocket_ECEF) {}

#endif
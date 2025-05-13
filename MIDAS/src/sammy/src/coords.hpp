#ifndef COORDS_EIGEN_HPP
#define COORDS_EIGEN_HPP

#include <Eigen/Dense>
#include <cmath>

using namespace std;

// WGS84 ellipsoid constants
constexpr double a = 6378137.0;           // Semi-major axis
constexpr double b = 6356752.314245;      // Semi-minor axis

// Normalize angle to [-pi, pi)
inline double norm_pi(double angle) {
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

// Convert GPS (latitude, longitude in degrees, altitude) to ECEF coordinates
inline Eigen::Vector3d GPS_to_ECEF(const Eigen::Vector3d& gps) {
    double lat = gps.x() * M_PI / 180.0;
    double lon = gps.y() * M_PI / 180.0;
    double alt = gps.z();

    double e2 = (a*a - b*b) / (a*a);
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));

    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = ((1 - e2) * N + alt) * sin(lat);

    return Eigen::Vector3d(x, y, z);
}

// Multiply a 3x3 rotation matrix by a 3-vector
inline Eigen::Vector3d matrixMult(const Eigen::Matrix3d& R, const Eigen::Vector3d& v) {
    return R * v;
}

// Convert ECEF coordinates to local ENU frame
// pos_turr_GPS: [lat (deg), lon (deg), alt]
// pos_turr_ECEF: ECEF coords of turret position
// rocket_ECEF: ECEF coords of rocket
inline Eigen::Vector3d ECEF_to_ENU(
    const Eigen::Vector3d& pos_turr_GPS_deg,
    const Eigen::Vector3d& pos_turr_ECEF,
    const Eigen::Vector3d& rocket_ECEF)
{
    double lat = pos_turr_GPS_deg.x() * M_PI / 180.0;
    double lon = pos_turr_GPS_deg.y() * M_PI / 180.0;

    // Define ENU basis vectors in ECEF frame
    Eigen::Vector3d e;
    e << -sin(lon), cos(lon), 0;
    Eigen::Vector3d n;
    n << -(lat) * cos(lon),
         -(lat) * sin(lon),
          (lat);
    Eigen::Vector3d u;
    u <<  cos(lat) * cos(lon),
          (lat) * sin(lon),
          (lat);

    // Assemble rotation matrix from ECEF to ENU
    Eigen::Matrix3d R;
    R.row(0) = e;
    R.row(1) = n;
    R.row(2) = u;

    // Compute relative vector in ECEF
    Eigen::Vector3d rel = rocket_ECEF - pos_turr_ECEF;

    // Rotate to ENU
    return R * rel;
}

#endif // COORDS_EIGEN_HPP

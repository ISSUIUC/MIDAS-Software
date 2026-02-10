#include <Eigen/Eigen>
/**************************** ROTATION FUNCTIONS ****************************/

// Used in ekf.cpp for ECEF and ENU conversions
#define A 6378137.0                      // Equatorial radius
#define F (1.0 / 298.257223563)          // Flattening factor
#define B (A * (1 - F))                  // Polar radius
#define E_SQ ((A * A - B * B) / (A * A)) // Eccentricity squared
#define pi 3.1415

/**
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vec Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
template <typename Angles>
void BodyToGlobal(Angles &angles_rad, Eigen::Matrix<float, 3, 1> &body_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0., 
            0., cos(angles_rad.roll), -sin(angles_rad.roll), 
            0., sin(angles_rad.roll), cos(angles_rad.roll);

    pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
            0., 1., 0.,
            -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

    yaw << cos(angles_rad.yaw), -sin(angles_rad.yaw), 0.,
            sin(angles_rad.yaw), cos(angles_rad.yaw), 0.,
            0., 0., 1.;

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    // temp = body_vec expressed in global frame (rotation_matrix * body_vec)
    Eigen::Vector3f temp = rotation_matrix * body_vec;
    body_vec(0, 0) = temp(0);  
    body_vec(1, 0) = temp(1); 
    body_vec(2, 0) = temp(2);  
}

/**
 * @brief Converts a vector in the global frame to the body frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param global_vec Vector for rotation in the global frame
 *
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the body frame
 */
template <typename Angles>
void GlobalToBody(Angles &angles_rad, Eigen::Matrix<float, 3, 1> &global_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;

    roll << cos(angles_rad.roll), -sin(angles_rad.roll), 0.,
        sin(angles_rad.roll), cos(angles_rad.roll), 0.,
        0., 0., 1.;

    // Pitch about Y (tilt forward/back)
    pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
        0., 1., 0.,
        -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

    // Yaw about X (turn around up axis)
    yaw << 1., 0., 0.,
        0., cos(angles_rad.yaw), -sin(angles_rad.yaw),
        0., sin(angles_rad.yaw), cos(angles_rad.yaw);

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix.transpose() * global_vec;
    Eigen::Matrix3f R_zup_to_xup;
    R_zup_to_xup << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    global_vec = (R_zup_to_xup * rotation_matrix).transpose() * global_vec;
}

/**
 * @brief Converts GPS (degrees) to ECEF (meters)
 * @return Vector of ECEF coordinates {X, Y, Z}
 */
inline std::vector<float> gps_to_ecef(float lat, float lon, float alt) {
    // Convert to radians
    lat *= pi / 180.0;
    lon *= pi / 180.0;

    double N = A / std::sqrt(1 - E_SQ * std::sin(lat) * std::sin(lat));

    float ecef_x = (N + alt) * std::cos(lat) * std::cos(lon);
    float ecef_y = (N + alt) * std::cos(lat) * std::sin(lon);
    float ecef_z = ((1 - E_SQ) * N + alt) * std::sin(lat);

    return {ecef_x, ecef_y, ecef_z};
}

/**
 * @brief Converts the current ECEF (meters) to ENU (meters) based on a reference in both ECEF (meters) and GPS (degrees)
 * @return Vector of ENU coordinates {East, North, Up}
 */
inline std::vector<float> ecef_to_enu(std::vector<float> curr_ecef, std::vector<float> ref_ecef, std::vector<float> ref_gps) {
    float ref_lat = ref_gps[0] * pi / 180.0;
    float ref_lon = ref_gps[1] * pi / 180.0;

    float dx = curr_ecef[0] - ref_ecef[0];
    float dy = curr_ecef[1] - ref_ecef[1];
    float dz = curr_ecef[2] - ref_ecef[2];

    float east  = - std::sin(ref_lon) * dx 
                  + std::cos(ref_lon) * dy;
    float north = - std::sin(ref_lat) * std::cos(ref_lon) * dx
                  - std::sin(ref_lat) * std::sin(ref_lon) * dy
                  + std::cos(ref_lat) * dz;
    float up    =   std::cos(ref_lat) * std::cos(ref_lon) * dx
                  + std::cos(ref_lat) * std::sin(ref_lon) * dy
                  + std::sin(ref_lat) * dz;

    return {east, north, up};
}

// void eulerToQuaternion(
//     float roll,
//     float pitch,
//     float yaw,
//     Eigen::Matrix<float, 4, 1> &quat)
// {

//     float cr = cos(roll * 0.5f);
//     float sr = sin(roll * 0.5f);
//     float cp = cos(pitch * 0.5f);
//     float sp = sin(pitch * 0.5f);
//     float cy = cos(yaw * 0.5f);
//     float sy = sin(yaw * 0.5f);

//     quat(0, 0) = cy * cp * cr + sy * sp * sr;
//     quat(1, 0) = cy * cp * sr - sy * sp * cr;
//     quat(2, 0) = sy * cp * sr + cy * sp * cr;
//     quat(3, 0) = sy * cp * cr - cy * sp * sr;
// }
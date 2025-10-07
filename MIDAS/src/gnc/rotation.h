#include <Eigen/Eigen>
/**************************** ROTATION FUNCTIONS ****************************/

/**
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vec Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
template <typename Angles>
void BodyToGlobal(Angles& angles, Eigen::Matrix<float, 3, 1> &body_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1., 0., 0.,
            0., cos(angles.roll), -sin(angles.roll),
            0., sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0., sin(angles.pitch),
             0., 1., 0., 
             -sin(angles.pitch), 0., cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0.,
           sin(angles.yaw), cos(angles.yaw), 0., 
           0., 0., 1.;

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix * body_vec;
    body_vec = temp;
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
void GlobalToBody(Angles& angles, Eigen::Matrix<float, 3, 1> &global_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << 1, 0, 0,
            0, cos(angles.roll), -sin(angles.roll),
            0, sin(angles.roll), cos(angles.roll);
    pitch << cos(angles.pitch), 0, sin(angles.pitch), 
             0, 1, 0,
             -sin(angles.pitch), 0, cos(angles.pitch);
    yaw << cos(angles.yaw), -sin(angles.yaw), 0, 
           sin(angles.yaw), cos(angles.yaw), 
           0, 0, 1;

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix.transpose() * global_vec;
    global_vec = temp;
}

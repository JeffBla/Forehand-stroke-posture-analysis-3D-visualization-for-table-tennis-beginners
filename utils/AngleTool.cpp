#include "AngleTool.h"

rp3d::Vector3 angleTool::AngleTool::QuaternionToEulerAngles(const rp3d::Quaternion &q) {
    rp3d::Vector3 angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.y = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

rp3d::Vector3 angleTool::AngleTool::EulerAnglesToDegree(const rp3d::Vector3 &euler_vtr) {
    return euler_vtr * (180 / M_PI);
}

float angleTool::AngleTool::EulerAnglesToDegree(float euler_angle) {
    return euler_angle * (180 / M_PI);
}

rp3d::Vector3 angleTool::AngleTool::DegreeToEulerAngles(const rp3d::Vector3 &degree_vtr) {
    return degree_vtr * (M_PI / 180);
}

float angleTool::AngleTool::DegreeToEulerAngles(float degree_angle) {
    return degree_angle * (M_PI / 180);
}


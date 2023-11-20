#include "AngleTool.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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

rp3d::Quaternion angleTool::AngleTool::rotate_local(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                                    const rp3d::Quaternion local_coordinate_quatern) {
    auto v = rotate_local_euler(angleX, angleY, angleZ, local_coordinate_quatern);
    return rp3d::Quaternion::fromEulerAngles(v.x, v.y, v.z);
}

rp3d::Vector3
angleTool::AngleTool::rotate_local_euler(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                         const rp3d::Quaternion local_coordinate_quatern) {
    auto trans = glm::mat4(1.0f);

    auto changedCoordinate_x = local_coordinate_quatern * default_axis_x;
    auto changedCoordinate_y = local_coordinate_quatern * default_axis_y;
    auto changedCoordinate_z = local_coordinate_quatern * default_axis_z;

    auto rotate_mat_x = glm::rotate(trans, angleX,
                                    glm::vec3(changedCoordinate_x.x, changedCoordinate_x.y, changedCoordinate_x.z));
    auto rotate_mat_y = glm::rotate(trans, angleY,
                                    glm::vec3(changedCoordinate_y.x, changedCoordinate_y.y, changedCoordinate_y.z));
    auto rotate_mat_z = glm::rotate(trans, angleZ,
                                    glm::vec3(changedCoordinate_z.x, changedCoordinate_z.y, changedCoordinate_z.z));

    auto rotate_quatern_x = glm::quat_cast(rotate_mat_x);
    auto rotate_quatern_y = glm::quat_cast(rotate_mat_y);
    auto rotate_quatern_z = glm::quat_cast(rotate_mat_z);

    auto angle_x = AngleTool::QuaternionToEulerAngles(
            rp3d::Quaternion{rotate_quatern_x.x, rotate_quatern_x.y, rotate_quatern_x.z, rotate_quatern_x.w});
    auto angle_y = AngleTool::QuaternionToEulerAngles(
            rp3d::Quaternion{rotate_quatern_y.x, rotate_quatern_y.y, rotate_quatern_y.z, rotate_quatern_y.w});
    auto angle_z = AngleTool::QuaternionToEulerAngles(
            rp3d::Quaternion{rotate_quatern_z.x, rotate_quatern_z.y, rotate_quatern_z.z, rotate_quatern_z.w});

    auto angle = angle_x + angle_y + angle_z;
    return angle;
}


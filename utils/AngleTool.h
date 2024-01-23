#ifndef TESTBED_ANGLETOOL_H
#define TESTBED_ANGLETOOL_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <reactphysics3d/reactphysics3d.h>

namespace angleTool {

    class AngleTool {
    public:
        inline static const rp3d::Vector3 default_axis_x{1, 0, 0};
        inline static const rp3d::Vector3 default_axis_y{0, 1, 0};
        inline static const rp3d::Vector3 default_axis_z{0, 0, 1};

        static rp3d::Vector3 QuaternionToEulerAngles(const rp3d::Quaternion &q);

        static rp3d::Vector3 EulerAnglesToDegree(const rp3d::Vector3 &euler_vtr);
        static float EulerAnglesToDegree(float euler_angle);

        static rp3d::Vector3 DegreeToEulerAngles(const rp3d::Vector3 &degree_vtr);

        static float DegreeToEulerAngles(float degree_angle);

        static rp3d::Quaternion rotate_local(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                             const rp3d::Quaternion local_coordinate_quatern);

        static rp3d::Vector3 rotate_local_euler(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                                const rp3d::Quaternion local_coordinate_quatern);
    };

}


#endif //TESTBED_ANGLETOOL_H

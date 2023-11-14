#ifndef TESTBED_ANGLETOOL_H
#define TESTBED_ANGLETOOL_H

#include <reactphysics3d/reactphysics3d.h>

namespace angleTool {

    class AngleTool {
    public:
        static rp3d::Vector3 QuaternionToEulerAngles(const rp3d::Quaternion &q);

        static rp3d::Vector3 EulerAnglesToDegree(const rp3d::Vector3 &euler_vtr);
        static float EulerAnglesToDegree(float euler_angle);

        static rp3d::Vector3 DegreeToEulerAngles(const rp3d::Vector3 &degree_vtr);

        static float DegreeToEulerAngles(float degree_angle);
    };

}


#endif //TESTBED_ANGLETOOL_H

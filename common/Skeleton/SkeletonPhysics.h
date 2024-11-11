#ifndef SKELETONPHYSICS_H
#define SKELETONPHYSICS_H

#include <list>
#include "PhysicsObject.h"
#include "openglframework.h"

using namespace std;

namespace skeleton {

    class SkeletonPhysics {
    private:

        rp3d::PhysicsCommon &physicsCommon;
        rp3d::PhysicsWorld *world;
        list<PhysicsObject *> &bodies;
        std::string meshPath;
        // Constants
        const float linearDamping = 0.02f;
        const float angularDamping = 0.02f;
        const float frictionCoeff = 0.4f;
        const float massDensity = 15.0f;
        const string cone_filepath = "cone_offset.obj";
        const openglframework::Color objectColor = openglframework::Color(0.0f, 0.68f, 0.99f, 1.0f);
        const openglframework::Color postureWrongColor = openglframework::Color(1.0f, 0.0f, 0.0f, 1.0f);

        void ConfigureBody(PhysicsObject *new_object, const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation);

    public:
        SkeletonPhysics(rp3d::PhysicsCommon &physicsCommon, rp3d::PhysicsWorld *world, list<PhysicsObject *> &bodies)
                : physicsCommon(physicsCommon), world(world), bodies(bodies) {};

        PhysicsObject *CreatePhysicsBody(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                                         const openglframework::Vector3 &size);

        PhysicsObject *
        CreatePhysicsBody(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, const float radius);

        void UpdateBoneTransform(PhysicsObject *target, const rp3d::Transform &boneTransform);

        list<PhysicsObject *> &GetPhysicsObjects() { return bodies; }
    };

}  // namespace skeleton

#endif
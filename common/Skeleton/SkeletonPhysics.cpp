#include "ConvexMesh.h"
#include "Sphere.h"
#include "SkeletonPhysics.h"

namespace skeleton {
    PhysicsObject *SkeletonPhysics::CreatePhysicsBody(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                                                      const openglframework::Vector3 &size) {
        auto *new_bone_object = new ConvexMesh(true, physicsCommon, world, meshPath + cone_filepath,
                                               {size.x, size.y, size.z});
        new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
        new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
        ConfigureBody(new_bone_object, pos, orientation);
        bodies.push_back(new_bone_object);

        return new_bone_object;
    }

    PhysicsObject *SkeletonPhysics::CreatePhysicsBody(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                                                      const float radius) {
        auto *new_bone_object = new Sphere(true, radius, physicsCommon, world, meshPath);
        new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
        new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
        ConfigureBody(new_bone_object, pos, orientation);
        bodies.push_back(new_bone_object);

        return new_bone_object;
    }

    void SkeletonPhysics::ConfigureBody(PhysicsObject *new_object, const rp3d::Vector3 &pos,
                                        const rp3d::Quaternion &orientation) {
        new_object->setTransform(rp3d::Transform(pos, orientation));

        // Set the box color
        new_object->setColor(objectColor);
        new_object->setSleepingColor(objectColor);

        new_object->getRigidBody()->updateMassPropertiesFromColliders();
        new_object->getRigidBody()->setLinearDamping(linearDamping);
        new_object->getRigidBody()->setAngularDamping(angularDamping);
        new_object->getRigidBody()->setType(rp3d::BodyType::KINEMATIC);
    }

    void SkeletonPhysics::UpdateBoneTransform(PhysicsObject *target, const rp3d::Transform &boneTransform) {
        target->setTransform(boneTransform);
    }
} // skeleton
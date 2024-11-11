#ifndef SKELETON_H
#define SKELETON_H

#include <reactphysics3d/reactphysics3d.h>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "Bone.h"
#include "BVH.h"
#include "MotionController.h"
#include "BoneHierarchy.h"
#include "SkeletonPhysics.h"
#include "PostureAnalyzer.h"
#include "JointTransformManager.h"

namespace skeleton {

    using namespace bone;
    using namespace rp3d;

    class Skeleton {
    private:
        SkeletonPhysics *physics;
        MotionController *motionController;
        BoneHierarchy *boneHierarchy;
        PostureAnalyzer *postureAnalyzer;
        BVH *bvh;

        Event<Bone *> bone_transform_changed;

        void initializeSkeleton();

        void UpdateSkeleton(const map<std::string, rp3d::Transform>&);
    public:
        Skeleton(PhysicsCommon &physicsCommon,
                 PhysicsWorld *physicsWorld, list<PhysicsObject *> &mPhysicsObjects,
                 std::string &meshPath,
                 BVH *bvh,
                 const Vector3 &initialPosition)
                : physics(new SkeletonPhysics(physicsCommon, physicsWorld, mPhysicsObjects)),
                  boneHierarchy(new BoneHierarchy(meshPath, bvh, rp3d::Vector3({0, 0, 0}))),
                  bvh(bvh), motionController(new MotionController(bvh, initialPosition)) {
            initializeSkeleton();
        }

        ~Skeleton();

        void SetJointRotation(const std::string &boneName, const Vector3 &rotation);

        void NextFrame();

    };

}  // namespace skeleton

#endif
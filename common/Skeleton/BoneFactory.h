#ifndef BONEFACTORY_H
#define BONEFACTORY_H

#include <reactphysics3d/reactphysics3d.h>

#include "SkeletonPhysics.h"
#include "BoneHierarchy.h"

namespace skeleton {

    class BoneFactory {
    public:
        static Bone *CreateSphericalBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                                         const rp3d::Quaternion &orientation,
                                         float radius, const Joint *joint, SkeletonPhysics *physics, BVH *bvh);

        static Bone *CreateConicalBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                                       const rp3d::Quaternion &orientation,
                                       const openglframework::Vector3 &size, const Joint *joint,
                                       SkeletonPhysics *physics, BVH *bvh);
    };

}  // namespace skeleton

#endif
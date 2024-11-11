#include "BoneFactory.h"

namespace skeleton {
    Bone *BoneFactory::CreateSphericalBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                                           const rp3d::Quaternion &orientation,
                                           float radius, const Joint *joint, SkeletonPhysics *physics, BVH *bvh) {
        PhysicsObject *bone_object = physics->CreatePhysicsBody(pos, orientation, radius);
        auto new_bone = new Bone(bone_name, bone_object, BoneType::CONE, pos, parent, orientation,
                                 rp3d::Quaternion::identity(), physics->GetPhysicsObjects(), bvh, joint);
        if (parent != nullptr) {
            parent->AppendChild(new_bone);
        }
        return new_bone;
    }

    Bone *BoneFactory::CreateConicalBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                                         const rp3d::Quaternion &orientation,
                                         const openglframework::Vector3 &size, const Joint *joint,
                                         SkeletonPhysics *physics, BVH *bvh) {
        PhysicsObject *bone_object = physics->CreatePhysicsBody(pos, orientation, size);
        auto new_bone = new Bone(bone_name, bone_object, BoneType::CONE, pos, parent, orientation,
                                 rp3d::Quaternion::identity(), physics->GetPhysicsObjects(), bvh, joint);
        if (parent != nullptr) {
            parent->AppendChild(new_bone);
        }
        return new_bone;
    }
} // skeleton
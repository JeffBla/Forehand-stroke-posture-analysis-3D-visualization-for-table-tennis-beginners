#include "BoneHierarchy.h"

namespace skeleton {

    Bone *BoneHierarchy::CreateBone(const float cone_size, const Joint *joint,
                                    SkeletonPhysics *physics) {
        Bone *new_bone = nullptr;
        if (joint->parents.empty()) {
            new_bone = BoneFactory::CreateSphericalBone(joint->name, nullptr, root_pos, rp3d::Quaternion::identity(),
                                                        root_radius, joint, physics, bvh);
        } else {
            float length = glm::length(glm::vec3{joint->offset[0], joint->offset[1], joint->offset[2]});
            // Prevent length from being 0
            if (length == 0) {
                length = 0.1;
            }
            length *= SCALE;
            new_bone = BoneFactory::CreateConicalBone(joint->name, bones[joint->parents.back()->name], root_pos,
                                          rp3d::Quaternion::identity(), {cone_size, length, cone_size},
                                          joint, physics, bvh);
        }
        bones[joint->name] = new_bone;
        return new_bone;
    }

    Bone *BoneHierarchy::GetBone(const string &bone_name) {
        return bones[bone_name];
    }

    void BoneHierarchy::NotifyTransformChange(Bone *bone) {

    }
} // skeleton
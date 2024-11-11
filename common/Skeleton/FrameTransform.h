#ifndef FRAMETRANSFORMS_H
#define FRAMETRANSFORMS_H

#include <vector>
#include <glm/glm.hpp>
#include "Bone.h"

namespace skeleton {

    class FrameTransform {
    public:
        string bone_name;
        glm::mat4 translation;
        glm::mat4 rotation;

        FrameTransform(const string &bone_name, const glm::mat4 &translation, const glm::mat4 &rotation)
                : bone_name(bone_name), translation(translation), rotation(rotation) {}

        FrameTransform(const string &bone_name)
                : FrameTransform(bone_name, glm::mat4(1.0), glm::mat4(1.0)) {}

        void ApplyParentTransform(const glm::vec3 &position, const glm::vec3 &angles,
                                  const Joint *parentJoint, BVH *bvhData);

        void ApplyRotation(const glm::vec3 &angle, ChannelEnum axis);

        void ApplyJointRotation(const glm::vec3 &jointPosition);

    };

}  // namespace skeleton

#endif
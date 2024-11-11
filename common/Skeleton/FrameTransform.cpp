#include "FrameTransform.h"

namespace skeleton {
    void FrameTransform::ApplyParentTransform(
            const glm::vec3 &position,
            const glm::vec3 &angles,
            const Joint *parentJoint,
            BVH *bvhData
    ) {
        this->translation = glm::translate(this->translation, position);

        const auto &rotationOrder = bvhData->GetRotationOrder(parentJoint->index);
        for (const auto &axis: rotationOrder) {
            this->ApplyRotation(angles, axis);
        }
    }

    void FrameTransform::ApplyRotation(const glm::vec3 &angle, ChannelEnum axis) {
        glm::vec3 axisVec3;
        float radians;

        switch (axis) {
            case X_ROTATION:
                radians = glm::radians(angle.x);
                axisVec3 = glm::vec3(1.0f, 0.0f, 0.0f);
                break;
            case Y_ROTATION:
                radians = glm::radians(angle.y);
                axisVec3 = glm::vec3(0.0f, 1.0f, 0.0f);
                break;
            case Z_ROTATION:
                radians = glm::radians(angle.z);
                axisVec3 = glm::vec3(0.0f, 0.0f, 1.0f);
                break;
        }

        this->rotation = glm::rotate(this->rotation, radians, axisVec3);
        this->translation = glm::rotate(this->translation, radians, axisVec3);
    }

    void FrameTransform::ApplyJointRotation(const glm::vec3 &jointPosition) {
        glm::vec3 orig(0.0f, -1.0f, 0.0f);
        glm::vec3 cross = glm::normalize(glm::cross(jointPosition, orig));

        if (glm::length(cross) > 0) {
            float angle = glm::pi<float>() - glm::acos(glm::dot(jointPosition, orig));
            this->rotation = glm::rotate(this->rotation, angle, cross);
            this->translation = glm::rotate(this->translation, angle, cross);
        } else if (jointPosition.x > 0) {
            float angle = glm::radians(180.0f);
            glm::vec3 axis(0.0f, 1.0f, 0.0f);
            this->rotation = glm::rotate(this->rotation, angle, axis);
            this->translation = glm::rotate(this->translation, angle, axis);
        }
    }

} // skeleton
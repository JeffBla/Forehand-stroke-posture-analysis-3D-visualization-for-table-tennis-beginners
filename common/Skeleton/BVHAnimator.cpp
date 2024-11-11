#include "BVHAnimator.h"

namespace skeleton {

    std::vector<FrameTransform *> BVHAnimator::ComputeFrameTransforms(uint32_t frameIndex) {
        std::vector<FrameTransform *> transforms;
        transforms.reserve(bvhData->GetNumJoint());

        bvhData->SetCurrentFrame(frameIndex);
        const auto &positions = bvhData->GetCurrentFramePositions();
        const auto &angles = bvhData->GetCurrentFrameAngles();

        for (int id = 0; id < bvhData->GetNumJoint(); id++) {
            auto joint = bvhData->GetJoint(id);
            auto *pFrameTransform = new FrameTransform(joint->name);

            ProcessParentTransforms(pFrameTransform, positions, angles, joint);
            ApplyHipTransform(pFrameTransform, joint->name);
            pFrameTransform->ApplyJointRotation(glm::normalize(positions[id]));

            transforms.push_back(pFrameTransform);
        }

        return transforms;
    }

    void BVHAnimator::ProcessParentTransforms(
            FrameTransform *transforms,
            const std::vector<glm::vec3> &positions,
            const std::vector<glm::vec3> &angles,
            const Joint *joint
    ) {
        for (const auto &parent: joint->parents) {
            glm::vec3 position = (parent->name != "hip") ?
                                 positions[parent->index] : initialPosition;

            transforms->ApplyParentTransform(position, angles[parent->index], parent, bvhData);
        }
    }

    void BVHAnimator::ApplyHipTransform(FrameTransform *transforms, const std::string &jointName) {
        if (jointName == "hip") {
            transforms->translation = glm::translate(
                    transforms->translation,
                    initialPosition
            );
        }
    }

}  // namespace skeleton
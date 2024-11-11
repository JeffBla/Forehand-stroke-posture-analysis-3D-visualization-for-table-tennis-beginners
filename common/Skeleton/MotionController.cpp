#include "MotionController.h"

namespace skeleton {
    const map<std::string, rp3d::Transform> & MotionController::InitializeAnimation() {
        currentFrame = 0;
        return ApplyFrame(currentFrame);
    }

    const map<std::string, rp3d::Transform> & MotionController::ComputerNextFrame() {
        currentFrame = (currentFrame + 1) % bvhData->GetNumFrame();
        return ApplyFrame(currentFrame);
    }

    const map<std::string, rp3d::Transform> & MotionController::ApplyFrame(uint32_t frameIndex) {
        currentFrame = frameIndex;
        auto transforms = bvhAnimator->ComputeFrameTransforms(currentFrame);

        // Convert and store transforms for physics update
        pendingTransforms.clear();
        for (const auto &transform: transforms) {
            glm::vec4 result_pos = transform->translation * glm::vec4(0.0, 0.0, 0.0, 1.0);
            glm::quat result_angle = glm::quat_cast(transform->rotation);

            // Convert to physics engine format
            rp3d::Vector3 physPosition(result_pos.x, result_pos.y, result_pos.z);
            rp3d::Quaternion physRotation(result_angle.x, result_angle.y, result_angle.z, result_angle.w);

            pendingTransforms[transform->bone_name] = rp3d::Transform(physPosition, physRotation);
        }
    }


} // skeleton
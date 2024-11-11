#ifndef BVHANIMATOR_H
#define BVHANIMATOR_H

#include <map>
#include <string>
#include <vector>
#include "Bone.h"
#include "BVH.h"
#include "FrameTransform.h"
#include "TransformApplicator.h"

using namespace bone;

namespace skeleton {

    class BVHAnimator {
    private:
        BVH *bvhData;
        std::map<std::string, Bone *> boneMap;
        std::vector<std::string> targetBones;
        glm::vec3 initialPosition;

        void ProcessParentTransforms(
                FrameTransform *transforms,
                const std::vector<glm::vec3> &positions,
                const std::vector<glm::vec3> &angles,
                const Joint *joint
        );

        void ApplyHipTransform(
                FrameTransform *transforms,
                const std::string &jointName
        );

    public:
        BVHAnimator(BVH *bvhData, rp3d::Vector3 initialPosition)
                : bvhData(bvhData), initialPosition(initialPosition.x, initialPosition.y, initialPosition.z) {};

        std::vector<FrameTransform *> ComputeFrameTransforms(uint32_t frameIndex);
    };

}  // namespace skeleton

#endif
#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "BVHAnimator.h"

namespace skeleton {

    class MotionController {
    private:
        BVHAnimator *bvhAnimator;
        BVH *bvhData;
        uint32_t currentFrame = 0;
        rp3d::Vector3 initialPosition{0, 0, 0};

        std::map<std::string, rp3d::Transform> pendingTransforms;

    public:
        MotionController(BVH *bvhData, rp3d::Vector3 initialPosition) : bvhData(bvhData), initialPosition(initialPosition) {
            bvhAnimator = new BVHAnimator(bvhData, initialPosition);
        };

        const map<std::string, rp3d::Transform> &InitializeAnimation();

        const map<std::string, rp3d::Transform> &ComputerNextFrame();

        const map<std::string, rp3d::Transform> &ApplyFrame(uint32_t frameIndex);

        const std::map<std::string, rp3d::Transform> &GetPendingTransforms() const {
            return pendingTransforms;
        }

        void SetInitialPosition(const rp3d::Vector3 &initialPosition) {
            this->initialPosition = initialPosition;
        }
    };

}  // namespace skeleton

#endif
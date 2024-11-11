#ifndef TRANSFORMAPPLICATOR_H
#define TRANSFORMAPPLICATOR_H

#include "Bone.h"
#include <glm/glm.hpp>

using namespace bone;

namespace skeleton {

    class TransformApplicator {
    public:
        void applyTransform(Bone *bone, const glm::vec3 &position, const rp3d::Quaternion &orientation);
        void updateVisualState(Bone *bone);
    };

}  // namespace skeleton

#endif
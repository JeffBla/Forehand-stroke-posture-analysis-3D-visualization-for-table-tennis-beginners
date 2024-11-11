#ifndef JOINTTRANSFORMMANAGER_H
#define JOINTTRANSFORMMANAGER_H

#include "Bone.h"
#include "Event.h"

using namespace bone;
using namespace event;

namespace skeleton {

    class JointTransformManager {
    private:
        Event<Bone *> transformNotifier;

    public:
        void setWorldRotation(Bone *bone, const rp3d::Quaternion &rotation);
        void setLocalRotation(Bone *bone, const rp3d::Quaternion &rotation);
        void propagateTransforms(Bone *bone);
    };

}  // namespace skeleton

#endif
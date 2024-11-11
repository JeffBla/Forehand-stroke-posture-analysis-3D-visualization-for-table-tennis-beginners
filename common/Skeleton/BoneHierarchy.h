#ifndef BONEHIERARCHY_H
#define BONEHIERARCHY_H

#include <map>
#include <string>
#include <vector>

#include "Bone.h"
#include "Event.h"
#include "SkeletonPhysics.h"
#include "BoneFactory.h"

using namespace bone;
using namespace event;

namespace skeleton {

    class BoneHierarchy {
    private:
        string meshPath;
        std::map<std::string, Bone *> boneMap;
        std::vector<std::string> targetBones;
        std::map<const string, Bone *> bones;
        Event<Bone *> transformChangeNotifier;
        BVH *bvh;

        rp3d::Vector3 root_pos{0, 0, 0};
        float root_radius = 0.2f;

        // Constants
        static const float SCALE;

    public:
        BoneHierarchy(string &meshPath, BVH *bvh, rp3d::Vector3 root_pos) : meshPath(meshPath), bvh(bvh),
                                                                            root_pos(root_pos) {};

        Bone *
        CreateBone(const float cone_size, const Joint *joint, SkeletonPhysics *physics);

        Bone *GetBone(const std::string &bone_name);

        void NotifyTransformChange(Bone *bone);
    };

}  // namespace skeleton

#endif  // BONEHIERARCHY_H
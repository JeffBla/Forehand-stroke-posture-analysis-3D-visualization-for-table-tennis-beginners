#ifndef TESTBED_BONE_H
#define TESTBED_BONE_H


#include <string>
#include <map>
#include <vector>
#include "PhysicsObject.h"

namespace bone {

    enum BoneType {
        CONE, SPHERE
    };

    class Bone {
    private:
        inline static const rp3d::Vector3 default_orientation{0, 1, 0};

        BoneType boneType;
        const std::string bone_name;
        PhysicsObject *bone_object;
        rp3d::Vector3 position;
        Bone *parent;
        std::map<std::string, Bone *> children;
    public:
        Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
             Bone *parent);

        ~Bone();

        void AppendChild(Bone *child);

        void UpdateChild();

        std::map<std::string, float> GetAngleWithNeighbor();

        const std::string &GetBoneName();

        PhysicsObject *GetPhysicsObject();

        rp3d::Vector3 &GetPosition();

        void SetPosition(rp3d::Vector3 &pos);

        BoneType GetBoneType() const;
    };

    inline void Bone::AppendChild(Bone *child) {
        children[child->bone_name] = child;
    };

    inline const std::string &Bone::GetBoneName() {
        return bone_name;
    }


    inline PhysicsObject *Bone::GetPhysicsObject() {
        return bone_object;
    };

    inline rp3d::Vector3 &Bone::GetPosition() {
        return position;
    }

    inline void Bone::SetPosition(rp3d::Vector3 &pos) {
        position = pos;
    }

    inline BoneType Bone::GetBoneType() const{
        return boneType;
    }
}


#endif //TESTBED_BONE_H

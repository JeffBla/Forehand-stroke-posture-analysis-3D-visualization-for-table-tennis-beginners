#ifndef TESTBED_BONE_H
#define TESTBED_BONE_H


#include <string>
#include <map>
#include "PhysicsObject.h"

namespace bone {

    enum BoneType{
        CONE, SPHERE
    };

    class Bone {
    private:
        inline static const rp3d::Vector3 default_orientation{0,1,0};

        BoneType boneType;
        std::string bone_name;
        PhysicsObject *bone_object;
        rp3d::Vector3 position;
        Bone *parent;
        std::map<std::string, Bone *> children;
    public:
        Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos, Bone *parent);

        ~Bone();

        void AppendChild(Bone *child);

        void UpdateChild();

        PhysicsObject *GetPhysicsObject();

        rp3d::Vector3 &GetPosition();
    };

    inline void Bone::AppendChild(Bone *child) {
        children[child->bone_name] = child;
    };

    inline PhysicsObject *Bone::GetPhysicsObject() {
        return bone_object;
    };

    inline rp3d::Vector3 &Bone::GetPosition() {
        return position;
    }
}


#endif //TESTBED_BONE_H

#ifndef TESTBED_BONE_H
#define TESTBED_BONE_H


#include <string>
#include <map>
#include "PhysicsObject.h"

namespace bone {

    class Bone {
    private:
        std::string bone_name;
        PhysicsObject *bone_object;
        Bone *parent;
        std::map<std::string, Bone *> children;
    public:
        Bone(const std::string &bone_name, PhysicsObject *bone_object, Bone *parent);

        ~Bone();

        inline void AppendChild(Bone *child) {
            children[child->bone_name] = child;
        };

        inline PhysicsObject *GetPhysicsObject(){
            return bone_object;
        };
    };
}

#endif //TESTBED_BONE_H

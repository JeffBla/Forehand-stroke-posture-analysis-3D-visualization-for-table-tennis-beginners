#include "Bone.h"

using namespace bone;

Bone::Bone(const std::string &bone_name,  PhysicsObject *bone_object,Bone *parent) : bone_name(bone_name), parent(parent),
                                                                               bone_object(bone_object) {}

Bone::~Bone() {

}


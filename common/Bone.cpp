#include "Bone.h"
#include "Sphere.h"
#include "ConvexMesh.h"

using namespace bone;

Bone::Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
           Bone *parent)
        : bone_name(bone_name), position(pos), parent(parent), bone_object(bone_object), boneType(boneType) {}

Bone::~Bone() {

}


void Bone::UpdateChild() {
    for (auto &[key, cBone]: children) {
        rp3d::Quaternion quatern = cBone->GetPhysicsObject()->getTransform().getOrientation();
        rp3d::Vector3 pos;
        auto cObject = cBone->GetPhysicsObject();

        // If the shape cBone is sphere, need offset
        float offset = 0;
        if (cBone->GetBoneType() == SPHERE) {
            offset = ((Sphere *) cObject)->GetRadius() / 2;
        }

        switch (boneType) { // check parent boneType
            case SPHERE:
                pos = position + bone_object->getTransform().getOrientation() * default_orientation *
                                 (((Sphere *) bone_object)->GetRadius() + offset);
                break;
            default:
            case CONE:
                pos = position + bone_object->getTransform().getOrientation() * default_orientation *
                                 (((ConvexMesh *) bone_object)->GetSize().y + offset);
        }

        cBone->SetPosition(pos);
        cBone->GetPhysicsObject()->setTransform(rp3d::Transform(pos, quatern));
        cBone->UpdateChild();
    }
}

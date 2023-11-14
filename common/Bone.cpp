#include "Bone.h"
#include "Sphere.h"
#include "ConvexMesh.h"
#include "AngleTool.h"

using namespace angleTool;
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
            offset = ((Sphere *) cObject)->GetRadius();
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

std::map<std::string, float> Bone::GetAngleWithNeighbor() {
    std::map<std::string, float> angles;

    rp3d::Quaternion myQuaternion = bone_object->getTransform().getOrientation();
    rp3d::Vector3 myOrientation = (myQuaternion * default_orientation).getUnit();

    rp3d::Quaternion otherQuatern;
    rp3d::Vector3 otherOrient;
    for (auto &[name, cBone]: children) {
        otherQuatern = cBone->GetPhysicsObject()->getTransform().getOrientation();
        otherOrient = (otherQuatern * default_orientation).getUnit();

        angles[name] = AngleTool::EulerAnglesToDegree(
                acos(otherOrient.dot(myOrientation))); // since the orientation vectors are unit len.
    }
    { // calculate angle between parent and itself
        otherQuatern = parent->GetPhysicsObject()->getTransform().getOrientation();
        otherOrient = (otherQuatern * default_orientation).getUnit();
        angles["parent"] = AngleTool::EulerAnglesToDegree(acos(otherOrient.dot(myOrientation)));
    }
    return angles;
}

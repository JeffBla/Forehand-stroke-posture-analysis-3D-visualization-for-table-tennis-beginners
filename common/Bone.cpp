#include "Bone.h"
#include "Sphere.h"
#include "ConvexMesh.h"
#include "AngleTool.h"

using namespace angleTool;
using namespace bone;

Bone::Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
           Bone *parent, rp3d::Quaternion &quatern)
        : bone_name(bone_name), position(pos), parent(parent), bone_object(bone_object), boneType(boneType),
          origin_quatern(quatern), init_quatern(quatern) {}

Bone::Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
           Bone *parent, const rp3d::Quaternion &quatern)
        : bone_name(bone_name), position(pos), parent(parent), bone_object(bone_object), boneType(boneType),
          origin_quatern(quatern), init_quatern(quatern) {}

Bone::~Bone() {

}

void Bone::UpdateChild(const rp3d::Quaternion &changedQuatern) {
    for (auto &[key, cBone]: children) {
        /// rotation
        auto old_init_euler = AngleTool::QuaternionToEulerAngles(cBone->GetInitQuaternion());
        auto old_origin_euler = AngleTool::QuaternionToEulerAngles(cBone->GetOriginQuaternion());
        auto old_rotation_euler = AngleTool::QuaternionToEulerAngles(
                cBone->GetPhysicsObject()->getTransform().getOrientation());

        auto change_euler = AngleTool::QuaternionToEulerAngles(changedQuatern);
        cBone->SetOriginQuaternion(rp3d::Quaternion::fromEulerAngles(change_euler.x + old_init_euler.x,
                                                                     change_euler.y + old_init_euler.y,
                                                                     change_euler.z + old_init_euler.z));
        auto new_origin_euler = AngleTool::QuaternionToEulerAngles(cBone->GetOriginQuaternion());
        auto new_quatern = rp3d::Quaternion::fromEulerAngles(
                old_rotation_euler.x - old_origin_euler.x + new_origin_euler.x,
                old_rotation_euler.y - old_origin_euler.y + new_origin_euler.y,
                old_rotation_euler.z - old_origin_euler.z + new_origin_euler.z);

        /// translation
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
        cBone->GetPhysicsObject()->setTransform(rp3d::Transform(pos, new_quatern));
        cBone->UpdateChild(changedQuatern);
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
    if (parent != nullptr) {// calculate angle between parent and itself
        otherQuatern = parent->GetPhysicsObject()->getTransform().getOrientation();
        otherOrient = (otherQuatern * default_orientation).getUnit();
        angles["parent"] = AngleTool::EulerAnglesToDegree(acos(otherOrient.dot(myOrientation)));
    }
    return angles;
}

#include "Bone.h"
#include "Sphere.h"
#include "ConvexMesh.h"
#include "AngleTool.h"

using namespace angleTool;
using namespace bone;

Bone::Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
           Bone *parent, rp3d::Quaternion &quatern, rp3d::Quaternion local_coordinate_quatern)
        : bone_name(bone_name), position(pos), parent(parent), bone_object(bone_object), boneType(boneType),
          origin_quatern(quatern), init_quatern(quatern), local_coordinate_quatern(local_coordinate_quatern),
          init_local_coordinate_quatern(local_coordinate_quatern) {}

Bone::Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
           Bone *parent, const rp3d::Quaternion &quatern, rp3d::Quaternion local_coordinate_quatern)
        : bone_name(bone_name), position(pos), parent(parent), bone_object(bone_object), boneType(boneType),
          origin_quatern(quatern), init_quatern(quatern), local_coordinate_quatern(local_coordinate_quatern),
          init_local_coordinate_quatern(local_coordinate_quatern) {}

Bone::~Bone() {

}


void Bone::SetJointRotation_local(rp3d::Vector3 &angle) {
    SetJointRotation_local(angle.x, angle.y, angle.z);
}


void Bone::SetJointRotation_local(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
    auto q = AngleTool::rotate_local(angleX, angleY, angleZ, local_coordinate_quatern);
    auto new_quatern = q * origin_quatern;

    bone_object->setTransform({position, new_quatern});
}

void Bone::UpdateChild(const rp3d::Quaternion &changedQuatern) {
    for (auto &[key, cBone]: children) {
        /// rotation
        auto parentChanged_euler = AngleTool::QuaternionToEulerAngles(origin_quatern * init_quatern.getInverse());
        auto changedQ_euler = AngleTool::QuaternionToEulerAngles(changedQuatern);
        auto new_origin_quatern = AngleTool::rotate_local(changedQ_euler.x + parentChanged_euler.x,
                                                          changedQ_euler.y + parentChanged_euler.y,
                                                          changedQ_euler.z + parentChanged_euler.z,
                                                          cBone->GetLocalCoordinateQuatern()) *
                                  cBone->GetInitQuaternion();
        auto old_rotation_euler = AngleTool::QuaternionToEulerAngles(
                cBone->GetPhysicsObject()->getTransform().getOrientation() * cBone->GetOriginQuaternion().getInverse());

        cBone->SetOriginQuaternion(new_origin_quatern);

        cBone->SetJointRotation_local(old_rotation_euler.x, old_rotation_euler.y, old_rotation_euler.z);

        auto new_local_coordinate_quatern = AngleTool::rotate_local(changedQ_euler.x + parentChanged_euler.x,
                                                                    changedQ_euler.y + parentChanged_euler.y,
                                                                    changedQ_euler.z + parentChanged_euler.z,
                                                                    cBone->GetLocalCoordinateQuatern()) *
                                            cBone->GetInitLocalCoordinateQuatern();
        cBone->SetLocalCoordinateQuatern(new_local_coordinate_quatern);

        auto new_quatern = cBone->GetPhysicsObject()->getTransform().getOrientation();

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

        // right now it is no good for manipulate skeleton. this is only for bvh
        cBone->UpdateChild(rp3d::Quaternion::identity());
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

#include <queue>


#include "Bone.h"
#include "Sphere.h"
#include "ConvexMesh.h"
#include "AngleTool.h"
#include "BVH.h"

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


rp3d::Quaternion
Bone::_SetJointRotation_bvh(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                            const bvh::Joint *bvh_joint) {
    auto result_local_coordinate_quatern = local_coordinate_quatern;
    rp3d::Quaternion rotation_q;
    auto bone_channel = bvh_joint->channels;
    for (auto &channel: bone_channel) {
        auto channel_type = channel->type;
        switch (channel_type) {
            case X_ROTATION:
                rotation_q = AngleTool::rotate_local(angleX, 0, 0, result_local_coordinate_quatern);
                result_local_coordinate_quatern = rotation_q * result_local_coordinate_quatern;
                break;
            case Y_ROTATION:
                rotation_q = AngleTool::rotate_local(0, angleY, 0, result_local_coordinate_quatern);
                result_local_coordinate_quatern = rotation_q * result_local_coordinate_quatern;
                break;
            case Z_ROTATION:
                rotation_q = AngleTool::rotate_local(0, 0, angleZ, result_local_coordinate_quatern);
                result_local_coordinate_quatern = rotation_q * result_local_coordinate_quatern;
                break;
            default:
                break;
        }
    }
    return rotation_q;
}

void Bone::SetJointRotation_bvh(rp3d::Vector3 &angle, const bvh::Joint *bvh_joint) {
    SetJointRotation_bvh(angle.x, angle.y, angle.z, bvh_joint);
}

void Bone::SetJointRotation_bvh(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                const bvh::Joint *bvh_joint) {
    local_angle = {angleX, angleY, angleZ};
    this->bvh_joint = bvh_joint;

    auto result_q = _SetJointRotation_bvh(angleX, angleY, angleZ, bvh_joint);
//    if (bone_name == "lThigh")
//        result_q = AngleTool::rotate_local(0, 0, rp3d::PI_RP3D / 20, local_coordinate_quatern) * result_q;
//    else if (bone_name == "rThigh")
//        result_q = AngleTool::rotate_local(0, 0, -rp3d::PI_RP3D / 20, local_coordinate_quatern) * result_q;
    auto new_quatern = result_q * origin_quatern;

    bone_object->setTransform({position, new_quatern});
}

void Bone::UpdateChild(const rp3d::Quaternion &changedQuatern) {
    for (auto &[key, cBone]: children) {
        /// rotation
        auto parentChanged_euler = AngleTool::QuaternionToEulerAngles(origin_quatern * init_quatern.getInverse());
        auto changedQ_euler = AngleTool::QuaternionToEulerAngles(changedQuatern);
        auto changedQ = rp3d::Quaternion::fromEulerAngles(changedQ_euler.x + parentChanged_euler.x,
                                                          changedQ_euler.y + parentChanged_euler.y,
                                                          changedQ_euler.z + parentChanged_euler.z);
        auto new_origin_quatern = changedQ * cBone->GetInitQuaternion();
        cBone->SetOriginQuaternion(new_origin_quatern);

        auto new_local_coordinate_quatern = changedQ * cBone->GetInitLocalCoordinateQuatern();
        cBone->SetLocalCoordinateQuatern(new_local_coordinate_quatern);

        if (bvh_joint != nullptr)
            cBone->SetJointRotation_bvh(cBone->GetLocalAngle(), bvh_joint);

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

        cBone->SetPosition(pos); // The position is for the object bone, not physic object
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

#include "Skeleton.h"
#include "AngleTool.h"

using namespace angleTool;
using namespace skeleton;
using namespace bone;


Bone *Skeleton::CreateBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                           const rp3d::Quaternion &orientation, const openglframework::Vector3 &size,
                           rp3d::decimal massDensity, const string &model_file,
                           const rp3d::Quaternion &local_coordinate_quatern) {
    PhysicsObject *boneObject = CreateBonePhysics(pos, orientation, size, massDensity, model_file);

    auto new_bone = new Bone(bone_name, boneObject, BoneType::CONE, pos, parent, orientation,
                             local_coordinate_quatern);
    if (parent != nullptr) {
        parent->AppendChild(new_bone);
    }
    return new_bone;
}

Bone *Skeleton::CreateBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos,
                           const rp3d::Quaternion &orientation, float radius, rp3d::decimal massDensity,
                           const rp3d::Quaternion &local_coordinate_quatern) {
    PhysicsObject *boneObject = CreateBonePhysics(pos, orientation, radius, massDensity);

    auto new_bone = new Bone(bone_name, boneObject, BoneType::SPHERE, pos, parent, orientation,
                             local_coordinate_quatern);

    if (parent != nullptr) {
        parent->AppendChild(new_bone);
    }
    return new_bone;
}

void
Skeleton::ConfigNewObject(PhysicsObject *new_object, const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation) {
    new_object->setTransform(rp3d::Transform(pos, orientation));

    // Set the box color
    new_object->setColor(objectColor);
    new_object->setSleepingColor(sleepingColor);

    new_object->getRigidBody()->updateMassPropertiesFromColliders();
    new_object->getRigidBody()->setLinearDamping(linearDamping);
    new_object->getRigidBody()->setAngularDamping(angularDamping);
    new_object->getRigidBody()->setType(rp3d::BodyType::KINEMATIC);
}

ConvexMesh *
Skeleton::CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                            const openglframework::Vector3 &size, rp3d::decimal massDensity,
                            const string &model_file) {
    auto *new_bone_object = new ConvexMesh(true, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath + model_file,
                                           {size.x, size.y, size.z});
    new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
    new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
    ConfigNewObject(new_bone_object, pos, orientation);
    mPhysicsObjects.push_back(new_bone_object);

    return new_bone_object;
}

Sphere *Skeleton::CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, float radius,
                                    rp3d::decimal massDensity) {
    auto *new_bone_object = new Sphere(true, radius, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
    new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
    ConfigNewObject(new_bone_object, pos, orientation);
    mPhysicsObjects.push_back(new_bone_object);

    return new_bone_object;
}

Skeleton::Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                   vector<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath, BVH *bvh)
        : mPhysicsCommon(mPhysicsCommon), mPhysicsWorld(mPhysicsWorld), mPhysicsObjects(mPhysicsObjects),
          mMeshFolderPath(mMeshFolderPath), bvh(bvh) {

    bvh_frame = 0;

    {
        rp3d::Vector3 ragdollPosition(0, 0, 0);
        rp3d::Vector3 defaultPosition(0, 0, 0);
        float mHip_radius = 0.2f;

        bones.clear();
        bones.resize(bvh->GetNumJoint());

        for (int id = 0; id < bvh->GetNumJoint(); id++) {
            auto joint = bvh->GetJoint(id);
            if (joint->parents.empty()) {
                // Root Joint
                bones[id] = CreateBone(joint->name, nullptr, ragdollPosition, rp3d::Quaternion::identity(),
                                       mHip_radius, 20, rp3d::Quaternion::identity());
            } else {
                float length = glm::length(glm::vec3{joint->offset[0], joint->offset[1], joint->offset[2]});
                if (length == 0) {
                    length = 0.1;
                }
                length *= SCALE;
                bones[id] = CreateBone(joint->name, bones[joint->parents.back()->index], defaultPosition,
                                       rp3d::Quaternion::identity(),
                                       {0.15, length, 0.15}, 9, "cone_offset_down.obj",
                                       rp3d::Quaternion::identity());
            }
        }
    }// Physic
}

Skeleton::~Skeleton() {
}

void Skeleton::SetJointRotation(Bone *bone, rp3d::Vector3 &angle) {
    SetJointRotation(bone, angle.x, angle.y, angle.z);
}

/// rotate worldly
void Skeleton::SetJointRotation(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
    auto old_eulerAngle = AngleTool::QuaternionToEulerAngles(bone->GetOriginQuaternion());
    auto new_quatern = rp3d::Quaternion::fromEulerAngles(angleX + old_eulerAngle.x, angleY + old_eulerAngle.y,
                                                         angleZ + old_eulerAngle.z);
    bone->GetPhysicsObject()->setTransform({bone->GetPosition(), new_quatern});

    // Event occur!!!
    bone_transform_changed.fire(bone);
    bone->UpdateChild(rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ));
}


void Skeleton::SetJointRotation_local(Bone *bone, rp3d::Vector3 &angle) {
    SetJointRotation_local(bone, angle.x, angle.y, angle.z);
}

void Skeleton::SetJointRotation_local(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
    bone->SetJointRotation_local(angleX, angleY, angleZ);

    // Event occur!!!
    bone_transform_changed.fire(bone);
    bone->UpdateChild(rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ));
}

void Skeleton::SetJointRotation_bvh(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                    const bvh::Joint *bone_bvh) {
    bone->SetJointRotation_bvh(angleX, angleY, angleZ, bone_bvh);

    // Event occur!!!
    bone_transform_changed.fire(bone);
    bone->UpdateChild(rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ));
}

Bone *Skeleton::FindBone(rp3d::RigidBody *body) {
    Bone *target = nullptr;
    for (auto bone: bones) {
        if (bone->GetPhysicsObject()->getRigidBody() == body) {
            target = bone;
            break;
        }
    }
    return target;
}

Bone *Skeleton::FindBone(const string &name) {
    Bone *target;
    for (auto bone: bones) {
        if (bone->GetBoneName() == name) {
            target = bone;
            break;
        }
    }
    return target;
}


void Skeleton::NextBvhMotion() {
    bvh_frame = (bvh_frame + 1) % bvh->GetNumFrame();
    ApplyBvhMotion(bvh_frame, bvh);
}

void Skeleton::ApplyBvhMotion(const int frame, BVH *other_bvh) {
    std::vector<rp3d::Vector3> positions(other_bvh->GetNumJoint());
    std::vector<rp3d::Vector3> angles(other_bvh->GetNumJoint());
    for (int i = 0; i < bones.size(); i++) {
        auto &pos = positions[i];
        auto &angle = angles[i];
        pos.setAllValues(other_bvh->GetJoint(i)->offset[0], other_bvh->GetJoint(i)->offset[1],
                         other_bvh->GetJoint(i)->offset[2]);

        auto bone_joint = other_bvh->GetJoint(i);
        for (auto channel: bone_joint->channels) {
            switch (channel->type) {
                case X_ROTATION:
                    angle.x = other_bvh->GetMotion(frame, channel->index);
                    break;
                case Y_ROTATION:
                    angle.y = other_bvh->GetMotion(frame, channel->index);
                    break;
                case Z_ROTATION:
                    angle.z = other_bvh->GetMotion(frame, channel->index);
                    break;
                case X_POSITION:
                    pos.x = other_bvh->GetMotion(frame, channel->index);
                    break;
                case Y_POSITION:
                    pos.y = other_bvh->GetMotion(frame, channel->index);
                    break;
                case Z_POSITION:
                    pos.z = other_bvh->GetMotion(frame, channel->index);
                    break;
            }
        }

        pos *= SCALE;
    }
    std::vector<glm::mat4> translations(bvh->GetNumJoint(), glm::mat4(1.0)), rotations(bvh->GetNumJoint(), glm::mat4(1.0));

    for (int id = 0; id < bvh->GetNumJoint(); id++) {
        auto joint = bvh->GetJoint(id);
        auto &parents = joint->parents;

        for (size_t parentIdx = 0; parentIdx < parents.size(); parentIdx++) {
            auto parent_joint = parents[parentIdx];
            const auto &pos = positions[parent_joint->index];
            const auto &angle = angles[parent_joint->index];
            // Move to eachn parent's position
            translations[id] =
                    glm::translate(translations[id], glm::vec3{pos.x, pos.y, pos.z});

            // Motion rotation
            const auto multiplyRotateMat = [&](ChannelEnum axis) {
                glm::vec3 axisVec3;
                float radians;
                switch (axis) {
                    case X_ROTATION:
                        radians = glm::radians(angle.x);
                        axisVec3 = glm::vec3(1.0, 0.0, 0.0);
                        break;
                    case Y_ROTATION:
                        radians = glm::radians(angle.y);
                        axisVec3 = glm::vec3(0.0, 1.0, 0.0);
                        break;
                    case Z_ROTATION:
                        radians = glm::radians(angle.z);
                        axisVec3 = glm::vec3(0.0, 0.0, 1.0);
                        break;
                }
                rotations[id] = glm::rotate(rotations[id], radians, axisVec3);
                translations[id] = glm::rotate(translations[id], radians, axisVec3);;
            };
            std::for_each(other_bvh->GetRotationOrder(parent_joint->index).begin(),
                          other_bvh->GetRotationOrder(parent_joint->index).end(), multiplyRotateMat);
        }

        glm::vec3 pos;
        pos = glm::vec3{positions[id].x, positions[id].y, positions[id].z};

        // Move to current joint's position
        translations[id] = glm::translate(translations[id], pos);

        // rotate current joint object to turn to parent
        pos = glm::normalize(pos);
        glm::vec3 orig = glm::vec3(0.0, -1.0, 0.0);
        glm::vec3 cross = glm::normalize(glm::cross(pos, orig));
        if (glm::length(cross) > 0) {
            rotations[id] = glm::rotate(rotations[id],
                                        glm::pi<float>() - glm::acos(glm::dot(pos, orig)), cross);
            translations[id] = glm::rotate(translations[id],
                                           glm::pi<float>() - glm::acos(glm::dot(pos, orig)), cross);
        } else if (pos.x > 0) {
            rotations[id] = glm::rotate(rotations[id], glm::radians(180.0f),
                                        glm::vec3(0.0, 1.0, 0.0));
            translations[id] = glm::rotate(translations[id], glm::radians(180.0f),
                                           glm::vec3(0.0, 1.0, 0.0));
        }
//        bone_transform_changed.fire(bones[id]);

        // use result
        auto bone = bones[id];
        if (bone->GetBoneName() == "head" || bone->GetBoneName() == "abdomen" || bone->GetBoneName() == "chest" ||
            bone->GetBoneName() == "neck" || bone->GetBoneName() == "rCollar" || bone->GetBoneName() == "rShldr" ||
            bone->GetBoneName() == "rForeArm" || bone->GetBoneName() == "rHand" || bone->GetBoneName() == "lCollar" ||
            bone->GetBoneName() == "lShldr" || bone->GetBoneName() == "lForeArm" || bone->GetBoneName() == "lHand" ||
            bone->GetBoneName() == "rButtock" || bone->GetBoneName() == "rThigh" || bone->GetBoneName() == "rShin" ||
            bone->GetBoneName() == "rFoot" || bone->GetBoneName() == "lButtock" || bone->GetBoneName() == "lThigh" ||
            bone->GetBoneName() == "lShin" || bone->GetBoneName() == "lFoot" || bone->GetBoneName() == "hip") {

            glm::vec4 result_pos = translations[id] * glm::vec4(0.0, 0.0, 0.0, 1.0);
            glm::quat result_angle = glm::quat_cast(rotations[id]);
            bone->GetPhysicsObject()->setTransform(
                    {{result_pos.x, result_pos.y, result_pos.z},
                     rp3d::Quaternion(result_angle.x, result_angle.y, result_angle.z, result_angle.w)});
        }
    }

}


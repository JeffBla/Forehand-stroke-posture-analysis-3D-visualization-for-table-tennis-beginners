#include "Skeleton.h"

#include <cmath>

using namespace skeleton;

Box *
Skeleton::CreateBone(const rp3d::Vector3 &pos, const openglframework::Vector3 &size) {
    Box *new_bone = new Box(true, size / 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    new_bone->setTransform(rp3d::Transform(pos, rp3d::Quaternion::identity()));
    new_bone->setColor(objectColor);
    new_bone->setSleepingColor(sleepingColor);
    new_bone->getCollider()->getMaterial().setMassDensity(9);
    new_bone->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
    new_bone->getRigidBody()->updateMassPropertiesFromColliders();
    new_bone->getRigidBody()->setLinearDamping(linearDamping);
    new_bone->getRigidBody()->setAngularDamping(angularDamping);
    new_bone->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(new_bone);
    return new_bone;
}

void
Skeleton::bvhJointToSkeleton(const rp3d::Vector3 &prevPos, const bvh::Joint *joint, float scale, Box *parentBone) {
    rp3d::Vector3 nexPos = prevPos + rp3d::Vector3(joint->offset[0], joint->offset[1], joint->offset[2]) * scale;
    // find body type
    BodyTypeGroup bodyType = BODY;
    if (joint->parent != nullptr) {
        for (auto parentJoint = joint->parent; parentJoint != nullptr; parentJoint = parentJoint->parent) {
            if (parentJoint->name.find("head") != string::npos) {
                bodyType = HEAD;
                break;
            } else if (parentJoint->name.find("Collar") != string::npos) {
                bodyType = HAND;
                break;
            } else if (parentJoint->name.find("Buttock") != string::npos) {
                bodyType = LEG;
                break;
            }
        }
    }


    Box *new_bone;
    if (joint->children.empty()) {
        openglframework::Vector3 boxSize(joint->site[0], joint->site[1], joint->site[2]);

        BoneSizeAdjust(boxSize);
        new_bone = CreateBone(nexPos + rp3d::Vector3(joint->site[0], joint->site[1], joint->site[2]) * scale / 2,
                              2 * boxSize * scale);

        ConfigBoneByGroup(new_bone, bodyType, parentBone);
    } else if (joint->children.size() == 1) {
        bvh::Joint *child = joint->children[0];
        openglframework::Vector3 boxSize(child->offset[0], child->offset[1], child->offset[2]);

        BoneSizeAdjust(boxSize);
        new_bone = CreateBone(nexPos + rp3d::Vector3(child->offset[0], child->offset[1], child->offset[2]) * scale / 2,
                              boxSize * scale * 2);

        ConfigBoneByGroup(new_bone, bodyType, parentBone);
    } else { // children more than one
        float center[3] = {0.0f, 0.0f, 0.0f};
        for (auto child: joint->children) {
            center[0] += child->offset[0];
            center[1] += child->offset[1];
            center[2] += child->offset[2];
        }
        center[0] /= joint->children.size() + 1;
        center[1] /= joint->children.size() + 1;
        center[2] /= joint->children.size() + 1;

        openglframework::Vector3 boxSize(center[0], center[1], center[2]);

        BoneSizeAdjust(boxSize);
        new_bone = CreateBone(nexPos + rp3d::Vector3(center[0], center[1], center[2]) * scale / 2, 2 * boxSize * scale);

        ConfigBoneByGroup(new_bone, bodyType, parentBone);

        for (auto child: joint->children) {

            new_bone = CreateBone(nexPos + rp3d::Vector3(center[0] + (child->offset[0] - center[0]) / 2,
                                                         center[1] + (child->offset[1] - center[1]) / 2,
                                                         center[2] + (child->offset[3] - center[3]) / 2) * scale,
                                  BoneSizeAdjust(child->offset[0] - center[0], child->offset[1] - center[1],
                                                 child->offset[2] - center[2]) * scale * 2);

            ConfigBoneByGroup(new_bone, bodyType, parentBone);
        }
    }

    for (auto &j: joint->children) {
        bvhJointToSkeleton(nexPos, j, scale, new_bone);
    }
}

Skeleton::Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                   vector<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath,
                   const bvh::Joint *hipJoint)
        : mPhysicsCommon(mPhysicsCommon), mPhysicsWorld(mPhysicsWorld), mPhysicsObjects(mPhysicsObjects),
          mMeshFolderPath(mMeshFolderPath) {

    bvhJointToSkeleton({0, 0, 0}, hipJoint, 0.4, nullptr);

}

Skeleton::~Skeleton() {
}


void Skeleton::initBodiesPositions() {
//    mHeadBox->setTransform(rp3d::Transform(mHeadPos, rp3d::Quaternion::identity()));
//    mChestBox->setTransform(rp3d::Transform(mChestPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mWaistBox->setTransform(rp3d::Transform(mWaistPos, rp3d::Quaternion::identity()));
//    mHipBox->setTransform(rp3d::Transform(mHipPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mLeftUpperArmBox->setTransform(rp3d::Transform(mLeftUpperArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mLeftLowerArmBox->setTransform(rp3d::Transform(mLeftLowerArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mLeftUpperLegBox->setTransform(rp3d::Transform(mLeftUpperLegPos, rp3d::Quaternion::identity()));
//    mLeftLowerLegBox->setTransform(rp3d::Transform(mLeftLowerLegPos, rp3d::Quaternion::identity()));
//    mRightUpperArmBox->setTransform(rp3d::Transform(mRightUpperArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mRightLowerArmBox->setTransform(rp3d::Transform(mRightLowerArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mRightUpperLegBox->setTransform(rp3d::Transform(mRightUpperLegPos, rp3d::Quaternion::identity()));
//    mRightLowerLegBox->setTransform(rp3d::Transform(mRightLowerLegPos, rp3d::Quaternion::identity()));
}

void Skeleton::BoneSizeAdjust(openglframework::Vector3 &vtr) {
    vtr.x = abs(vtr.x);
    vtr.y = abs(vtr.y);
    vtr.z = abs(vtr.z);
    if (vtr.x <= 0.001) {
        vtr.x = 1;
    }
    if (vtr.y <= 0.001) {
        vtr.y = 1;
    }
    if (vtr.z <= 0.001) {
        vtr.z = 1;
    }
}

openglframework::Vector3 Skeleton::BoneSizeAdjust(float x, float y, float z) {
    x = abs(x);
    y = abs(y);
    z = abs(z);
    if (x <= 0.001) {
        x = 1;
    }
    if (y <= 0.001) {
        y = 1;
    }
    if (z <= 0.001) {
        z = 1;
    }
    return {x, y, z};
}

void Skeleton::ConfigBoneByGroup(Box *new_bone, Skeleton::BodyTypeGroup bodyType, Box *parentBone) {
    if (parentBone != nullptr) {
        rp3d::RigidBody *body_new_bone = new_bone->getRigidBody();
        rp3d::RigidBody *body_parentBone = parentBone->getRigidBody();

        if (bodyType == BODY) {
            body_new_bone->setType(rp3d::BodyType::STATIC);
        } else if (bodyType == HEAD) {
            body_new_bone->setType(rp3d::BodyType::STATIC);
        } else if (bodyType == HAND) {
            body_new_bone->setType(rp3d::BodyType::DYNAMIC);

            rp3d::HingeJointInfo bodyJointInfo(body_new_bone, body_parentBone,
                                               body_new_bone->getTransform().getPosition() - new_bone->GetSize_rp3d(),
                                               {0, 0, 1});
            bodyJointInfo.isCollisionEnabled = false;
            auto joint = dynamic_cast<rp3d::HingeJoint *>( mPhysicsWorld->createJoint(bodyJointInfo));

            joint->enableLimit(true);
            joint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
            joint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);
        } else { // bodyType == LEG
            body_new_bone->setType(rp3d::BodyType::STATIC);
        }
    }
}



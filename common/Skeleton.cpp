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
    return nullptr;
}

void
Skeleton::bvhJointToSkeleton(const rp3d::Vector3 &prevPos, bvh::Joint *joint, float scale) {
    Box *new_bone;
    rp3d::Vector3 nexPos = prevPos + rp3d::Vector3(joint->offset[0], joint->offset[1], joint->offset[2]);
    if (joint->children.empty()) {
        new_bone = CreateBone(nexPos,
                              openglframework::Vector3(joint->site[0], joint->site[1], joint->site[2]) * scale);
        mPhysicsObjects.push_back(new_bone);
    } else if (joint->children.size() == 1) {
        bvh::Joint *child = joint->children[0];
        new_bone = CreateBone(nexPos,
                              openglframework::Vector3(child->offset[0], child->offset[1], child->offset[2]) * scale);
        mPhysicsObjects.push_back(new_bone);
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

        new_bone = CreateBone(nexPos,
                              openglframework::Vector3(center[0], center[1], center[2]) * scale);
        mPhysicsObjects.push_back(new_bone);

        for (int i = 0; i < joint->children.size(); i++) {
            bvh::Joint *child = joint->children[i];
            new_bone = CreateBone(nexPos,
                                  openglframework::Vector3(child->offset[0] - center[0], child->offset[1] - center[1],
                                                           child->offset[2] - center[2]) * scale);
            mPhysicsObjects.push_back(new_bone);

//            RenderBone(center[0] * scale, center[1] * scale, center[2] * scale,
//                       child->offset[0] * scale, child->offset[1] * scale, child->offset[2] * scale);
        }
    }

    for (auto &j: joint->children) {
        bvhJointToSkeleton(nexPos, j, scale);
    }
}

Skeleton::Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld, std::string &mMeshFolderPath,
                   bvh::Joint *hipJoint) : mPhysicsCommon(mPhysicsCommon), mPhysicsWorld(mPhysicsWorld),
                                           mMeshFolderPath(mMeshFolderPath) {

    // --------------- Create the hips Box --------------- //
    bvhJointToSkeleton(rp3d::Vector3(0, 0, 0), hipJoint, 0.2);


//    // --------------- Create the waist Box --------------- //
//    mWaistPos = mHipPos + rp3d::Vector3(0, 2, 0);
//    mWaistBox = new Box(true, 1, 1.5, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mWaistBox->setTransform(rp3d::Transform(mWaistPos, rp3d::Quaternion::identity()));
//    mWaistBox->setColor(objectColor);
//    mWaistBox->setSleepingColor(sleepingColor);
//    mWaistBox->getCollider()->getMaterial().setMassDensity(9);
//    mWaistBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mWaistBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mWaistBox->getRigidBody()->setLinearDamping(linearDamping);
//    mWaistBox->getRigidBody()->setAngularDamping(angularDamping);
//    mWaistBox->getRigidBody()->setType(rp3d::BodyType::STATIC);
//    mPhysicsObjects.push_back(mWaistBox);
//
//    // --------------- Create the chest Box --------------- //
//    mChestPos = mWaistPos + rp3d::Vector3(0, 2, 0);
//    mChestBox = new Box(true, 1, 1.5, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mChestBox->setTransform(rp3d::Transform(mChestPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mChestBox->setColor(objectColor);
//    mChestBox->setSleepingColor(sleepingColor);
//    mChestBox->getCollider()->getMaterial().setMassDensity(9);
//    mChestBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mChestBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mChestBox->getRigidBody()->setLinearDamping(linearDamping);
//    mChestBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mChestBox);
//
//    // --------------- Create the head box --------------- //
//    mHeadPos = mChestPos + rp3d::Vector3(0, 1.75, 0);
//    mHeadBox = new Sphere(true, 0.75f, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mHeadBox->setTransform(rp3d::Transform(mHeadPos, rp3d::Quaternion::identity()));
//    mHeadBox->setColor(objectColor);
//    mHeadBox->setSleepingColor(sleepingColor);
//    mHeadBox->getCollider()->getMaterial().setMassDensity(7);
//    mHeadBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mHeadBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mHeadBox->getRigidBody()->setLinearDamping(linearDamping);
//    mHeadBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mHeadBox);
//
//    // --------------- Create the left upper arm Box --------------- //
//    mLeftUpperArmPos = mChestPos + rp3d::Vector3(2.25, 0, 0);
//    mLeftUpperArmBox = new Box(true, 0.5, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mLeftUpperArmBox->setTransform(rp3d::Transform(mLeftUpperArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mLeftUpperArmBox->setColor(objectColor);
//    mLeftUpperArmBox->setSleepingColor(sleepingColor);
//    mLeftUpperArmBox->getCollider()->getMaterial().setMassDensity(8);
//    mLeftUpperArmBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mLeftUpperArmBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mLeftUpperArmBox->getRigidBody()->setLinearDamping(linearDamping);
//    mLeftUpperArmBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mLeftUpperArmBox);
//
//    // --------------- Create the left lower arm Box --------------- //
//    mLeftLowerArmPos = mLeftUpperArmPos + rp3d::Vector3(2.5, 0, 0);
//    mLeftLowerArmBox = new Box(true, 0.5, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mLeftLowerArmBox->setTransform(rp3d::Transform(mLeftLowerArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mLeftLowerArmBox->setColor(objectColor);
//    mLeftLowerArmBox->setSleepingColor(sleepingColor);
//    mLeftLowerArmBox->getCollider()->getMaterial().setMassDensity(8);
//    mLeftLowerArmBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mLeftLowerArmBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mLeftLowerArmBox->getRigidBody()->setLinearDamping(linearDamping);
//    mLeftLowerArmBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mLeftLowerArmBox);
//
//    // --------------- Create the left upper leg Box --------------- //
//    mLeftUpperLegPos = mHipPos + rp3d::Vector3(0.8, -1.5, 0);
//    mLeftUpperLegBox = new Box(true, 0.75, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mLeftUpperLegBox->setTransform(rp3d::Transform(mLeftUpperLegPos, rp3d::Quaternion::identity()));
//    mLeftUpperLegBox->setColor(objectColor);
//    mLeftUpperLegBox->setSleepingColor(sleepingColor);
//    mLeftUpperLegBox->getCollider()->getMaterial().setMassDensity(8);
//    mLeftUpperLegBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mLeftUpperLegBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mLeftUpperLegBox->getRigidBody()->setLinearDamping(linearDamping);
//    mLeftUpperLegBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mLeftUpperLegBox);
//
//    // --------------- Create the left lower leg Box --------------- //
//    mLeftLowerLegPos = mLeftUpperLegPos + rp3d::Vector3(0, -3, 0);
//    mLeftLowerLegBox = new Box(true, 0.5, 3, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mLeftLowerLegBox->setTransform(rp3d::Transform(mLeftLowerLegPos, rp3d::Quaternion::identity()));
//    mLeftLowerLegBox->setColor(objectColor);
//    mLeftLowerLegBox->setSleepingColor(sleepingColor);
//    mLeftLowerLegBox->getCollider()->getMaterial().setMassDensity(8);
//    mLeftLowerLegBox->getCollider()->getMaterial().setFrictionCoefficient(0.3);
//    mLeftLowerLegBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mLeftLowerLegBox->getRigidBody()->setLinearDamping(linearDamping);
//    mLeftLowerLegBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mLeftLowerLegBox);
//
//    // --------------- Create the right upper arm Box --------------- //
//    mRightUpperArmPos = mChestPos + rp3d::Vector3(-2.25, 0, 0);
//    mRightUpperArmBox = new Box(true, 0.5, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mRightUpperArmBox->setTransform(rp3d::Transform(mRightUpperArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mRightUpperArmBox->setColor(objectColor);
//    mRightUpperArmBox->setSleepingColor(sleepingColor);
//    mRightUpperArmBox->getCollider()->getMaterial().setMassDensity(8);
//    mRightUpperArmBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mRightUpperArmBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mRightUpperArmBox->getRigidBody()->setLinearDamping(linearDamping);
//    mRightUpperArmBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mRightUpperArmBox);
//
//    // --------------- Create the right lower arm Box --------------- //
//    mRightLowerArmPos = mRightUpperArmPos + rp3d::Vector3(-2.5, 0, 0);
//    mRightLowerArmBox = new Box(true, 0.5, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mRightLowerArmBox->setTransform(rp3d::Transform(mRightLowerArmPos, rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
//    mRightLowerArmBox->setColor(objectColor);
//    mRightLowerArmBox->setSleepingColor(sleepingColor);
//    mRightLowerArmBox->getCollider()->getMaterial().setMassDensity(8);
//    mRightLowerArmBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mRightLowerArmBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mRightLowerArmBox->getRigidBody()->setLinearDamping(linearDamping);
//    mRightLowerArmBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mRightLowerArmBox);
//
//    // --------------- Create the right upper leg Box --------------- //
//    mRightUpperLegPos = mHipPos + rp3d::Vector3(-0.8, -1.5, 0);
//    mRightUpperLegBox = new Box(true, 0.75, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mRightUpperLegBox->setTransform(rp3d::Transform(mRightUpperLegPos, rp3d::Quaternion::identity()));
//    mRightUpperLegBox->setColor(objectColor);
//    mRightUpperLegBox->setSleepingColor(sleepingColor);
//    mRightUpperLegBox->getCollider()->getMaterial().setMassDensity(8);
//    mRightUpperLegBox->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
//    mRightUpperLegBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mRightUpperLegBox->getRigidBody()->setLinearDamping(linearDamping);
//    mRightUpperLegBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mRightUpperLegBox);
//
//    // --------------- Create the right lower leg Box --------------- //
//    mRightLowerLegPos = mRightUpperLegPos + rp3d::Vector3(0, -3, 0);
//    mRightLowerLegBox = new Box(true, 0.5, 3, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
//    mRightLowerLegBox->setTransform(rp3d::Transform(mRightLowerLegPos, rp3d::Quaternion::identity()));
//    mRightLowerLegBox->setColor(objectColor);
//    mRightLowerLegBox->setSleepingColor(sleepingColor);
//    mRightLowerLegBox->getCollider()->getMaterial().setMassDensity(8);
//    mRightLowerLegBox->getCollider()->getMaterial().setFrictionCoefficient(0.3);
//    mRightLowerLegBox->getRigidBody()->updateMassPropertiesFromColliders();
//    mRightLowerLegBox->getRigidBody()->setLinearDamping(linearDamping);
//    mRightLowerLegBox->getRigidBody()->setAngularDamping(angularDamping);
//    mPhysicsObjects.push_back(mRightLowerLegBox);
//
//    // --------------- Create the joint between head and chest --------------- //
//
//    // Create the joint info object
//    rp3d::RigidBody* body1 = mHeadBox->getRigidBody();
//    rp3d::RigidBody* body2 = mChestBox->getRigidBody();
//    rp3d::BallAndSocketJointInfo jointInfo1(body1, body2, mHeadPos + rp3d::Vector3(0, -0.75, 0));
//    jointInfo1.isCollisionEnabled = false;
//    mHeadChestJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo1));
//    mHeadChestJoint->setConeLimitHalfAngle(40.0 * rp3d::PI_RP3D / 180.0);
//    mHeadChestJoint->enableConeLimit(true);
//
//    // --------------- Create the joint between chest and left upper arm --------------- //
//
//    // Create the joint info object
//    body1 = mChestBox->getRigidBody();
//    body2 = mLeftUpperArmBox->getRigidBody();
//    rp3d::BallAndSocketJointInfo jointInfo2(body1, body2, mLeftUpperArmPos + rp3d::Vector3(-1, 0, 0));
//    jointInfo2.isCollisionEnabled = false;
//    mChestLeftUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo2));
//    mChestLeftUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
//    mChestLeftUpperArmJoint->enableConeLimit(true);
//
//    // --------------- Create the joint between left upper arm and left lower arm  --------------- //
//
//    // Create the joint info object
//    body1 = mLeftUpperArmBox->getRigidBody();
//    body2 = mLeftLowerArmBox->getRigidBody();
//    rp3d::Vector3 joint2WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::Vector3 joint2WorldAxis(0, 0, 1);
//    rp3d::HingeJointInfo jointInfo3(body1, body2, joint2WorldAnchor, joint2WorldAxis);
//    jointInfo3.isCollisionEnabled = false;
//    mLeftUpperLeftLowerArmJoint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo3));
//    mLeftUpperLeftLowerArmJoint->enableLimit(true);
//    mLeftUpperLeftLowerArmJoint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
//    mLeftUpperLeftLowerArmJoint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);
//
//    // --------------- Create the joint between chest and waist  --------------- //
//
//    // Create the joint info object
//    body1 = mChestBox->getRigidBody();
//    body2 = mWaistBox->getRigidBody();
//    rp3d::Vector3 jointChestWaistWorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::FixedJointInfo jointChestWaistInfo(body1, body2, jointChestWaistWorldAnchor);
//    jointChestWaistInfo.isCollisionEnabled = false;
//    mChestWaistJoint = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointChestWaistInfo));
//
//    // --------------- Create the joint between waist and hips  --------------- //
//
//    // Create the joint info object
//    body1 = mWaistBox->getRigidBody();
//    body2 = mHipBox->getRigidBody();
//    rp3d::Vector3 jointWaistHipsWorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::FixedJointInfo jointWaistHipsInfo(body1, body2, jointWaistHipsWorldAnchor);
//    jointWaistHipsInfo.isCollisionEnabled = false;
//    mWaistHipsJoint = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointWaistHipsInfo));
//
//    // --------------- Create the joint between hip and left upper leg --------------- //
//
//    // Create the joint info object
//    body1 = mHipBox->getRigidBody();
//    body2 = mLeftUpperLegBox->getRigidBody();
//    rp3d::BallAndSocketJointInfo jointInfo4(body1, body2, mHipPos + rp3d::Vector3(0.8, 0, 0));
//    jointInfo4.isCollisionEnabled = false;
//    mHipLeftUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo4));
//    mHipLeftUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
//    mHipLeftUpperLegJoint->enableConeLimit(true);
//
//    // --------------- Create the joint between left upper leg and left lower leg  --------------- //
//
//    // Create the joint info object
//    body1 = mLeftUpperLegBox->getRigidBody();
//    body2 = mLeftLowerLegBox->getRigidBody();
//    rp3d::Vector3 joint5WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::Vector3 joint5WorldAxis(1, 0, 0);
//    const rp3d::decimal joint5MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
//    const rp3d::decimal joint5MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
//    rp3d::HingeJointInfo jointInfo5(body1, body2, joint5WorldAnchor, joint5WorldAxis, joint5MinAngle, joint5MaxAngle);
//    jointInfo5.isCollisionEnabled = false;
//    mLeftUpperLeftLowerLegJoint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo5));
//
//    // --------------- Create the joint between chest and right upper arm --------------- //
//
//    // Create the joint info object
//    body1 = mChestBox->getRigidBody();
//    body2 = mRightUpperArmBox->getRigidBody();
//    rp3d::BallAndSocketJointInfo jointInfo6(body1, body2, mRightUpperArmPos + rp3d::Vector3(1, 0, 0));
//    jointInfo6.isCollisionEnabled = false;
//    mChestRightUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo6));
//    mChestRightUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
//    mChestRightUpperArmJoint->enableConeLimit(true);
//
//    // --------------- Create the joint between right upper arm and right lower arm  --------------- //
//
//    // Create the joint info object
//    body1 = mRightUpperArmBox->getRigidBody();
//    body2 = mRightLowerArmBox->getRigidBody();
//    rp3d::Vector3 joint7WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::Vector3 joint7WorldAxis(0, 0, 1);
//    rp3d::HingeJointInfo jointInfo7(body1, body2, joint7WorldAnchor, joint7WorldAxis);
//    jointInfo7.isCollisionEnabled = false;
//    mRightUpperRightLowerArmJoint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo7));
//    mRightUpperRightLowerArmJoint->enableLimit(true);
//    mRightUpperRightLowerArmJoint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
//    mRightUpperRightLowerArmJoint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);
//
//    // --------------- Create the joint between hips and right upper leg --------------- //
//
//    // Create the joint info object
//    body1 = mHipBox->getRigidBody();
//    body2 = mRightUpperLegBox->getRigidBody();
//    rp3d::BallAndSocketJointInfo jointInfo8(body1, body2, mHipPos + rp3d::Vector3(-0.8, 0, 0));
//    jointInfo8.isCollisionEnabled = false;
//    mHipRightUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo8));
//    mHipRightUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
//    mHipRightUpperLegJoint->enableConeLimit(true);
//
//    // --------------- Create the joint between right upper leg and right lower leg  --------------- //
//
//    // Create the joint info object
//    body1 = mRightUpperLegBox->getRigidBody();
//    body2 = mRightLowerLegBox->getRigidBody();
//    rp3d::Vector3 joint9WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
//    rp3d::Vector3 joint9WorldAxis(1, 0, 0);
//    const rp3d::decimal joint9MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
//    const rp3d::decimal joint9MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
//    rp3d::HingeJointInfo jointInfo9(body1, body2, joint9WorldAnchor, joint9WorldAxis, joint9MinAngle, joint9MaxAngle);
//    jointInfo9.isCollisionEnabled = false;
//    mRightUpperRightLowerLegJoint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo9));
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


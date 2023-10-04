#include "Skeleton.h"

#include <cmath>

using namespace skeleton;
using namespace bone;

Bone *Skeleton::CreateBone(const string &bone_name, PhysicsObject *bone_object, Bone *parent) {
    auto new_bone = new Bone(bone_name, bone_object, parent);

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

Sphere *Skeleton::CreateBonePhysics_Sphere(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, float radius,
                                           rp3d::decimal massDensity) {
    auto *new_bone_object = new Sphere(true, radius, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
    new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
    ConfigNewObject(new_bone_object, pos, orientation);
    mPhysicsObjects.push_back(new_bone_object);

    return new_bone_object;
}


Box *Skeleton::CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                                 const openglframework::Vector3 &size, rp3d::decimal massDensity) {
    auto *new_bone_object = new Box(true, size, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    new_bone_object->getCollider()->getMaterial().setMassDensity(massDensity);
    new_bone_object->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
    ConfigNewObject(new_bone_object, pos, orientation);
    mPhysicsObjects.push_back(new_bone_object);

    return new_bone_object;
}

Skeleton::Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                   vector<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath,
                   const bvh::Joint *hipJoint)
        : mPhysicsCommon(mPhysicsCommon), mPhysicsWorld(mPhysicsWorld), mPhysicsObjects(mPhysicsObjects),
          mMeshFolderPath(mMeshFolderPath) {

    rp3d::Vector3 ragdollPosition(0, 0, 0);

    // --------------- Create the hips Sphere --------------- //
    mHipPos = ragdollPosition;
    mHipObject = CreateBonePhysics_Sphere(mHipPos, rp3d::Quaternion::identity(), 0.5f, 20);
//    mHip->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // handle bone hierarchy
    mHipBone = CreateBone("hip", mHipObject, nullptr);

    // --------------- Create the waist Cone --------------- //
    mWaistPos = mHipPos + rp3d::Vector3(0, 2, 0);
    mWaistObject = CreateBonePhysics(mWaistPos, rp3d::Quaternion::identity(), {0.2, 1.5, 0.2}, 9, "waist");

    mWaistBone = CreateBone("waist", mWaistObject, mHipBone);
    // --------------- Create the chest Cone --------------- //
    mChestPos = mWaistPos + rp3d::Vector3(0, 2, 0);
    mChestObject = CreateBonePhysics(mChestPos, rp3d::Quaternion::identity(), {0.2, 1.5, 0.2}, 9, "chest");

    mChestBone = CreateBone("chest", mChestObject, mWaistBone);
    // --------------- Create the head Sphere --------------- //
    mHeadPos = mChestPos + rp3d::Vector3(0, 1.75, 0);
    mHeadObject = CreateBonePhysics_Sphere(mHeadPos, rp3d::Quaternion::identity(), 0.75f, 7);

    mHeadBone = CreateBone("head", mHeadObject, mChestBone);
    // --------------- Create the left upper arm Cone --------------- //
    mLeftUpperArmPos = mChestPos + rp3d::Vector3(2.25, 0, 0);
    mLeftUpperArmObject = CreateBonePhysics(mLeftUpperArmPos,
                                            rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 2.0),
                                            {0.2, 2, 0.2}, 8, "leftUpperArm");

    mLeftUpperArmBone = CreateBone("leftUpperArm", mLeftUpperArmObject, mChestBone);
    // --------------- Create the left lower arm Cone --------------- //
    mLeftLowerArmPos = mLeftUpperArmPos + rp3d::Vector3(2.5, 0, 0);
    mLeftLowerArmObject = CreateBonePhysics(mLeftLowerArmPos,
                                            rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 2.0),
                                            {0.2, 2, 0.2}, 8, "leftLowerArm");

    mLeftLowerArmBone = CreateBone("leftLowerArm", mLeftLowerArmObject, mLeftUpperArmBone);
    // --------------- Create the left upper leg Cone --------------- //
    mLeftUpperLegPos = mHipPos + rp3d::Vector3(0.8, -1.5, 0);
    mLeftUpperLegObject = CreateBonePhysics(mLeftUpperLegPos,
                                            rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                            {0.2, 2, 0.2}, 8, "leftUpperLeg");

    mLeftUpperLegBone = CreateBone("leftUpperLeg", mLeftUpperLegObject, mHipBone);
    // --------------- Create the left lower leg Cone --------------- //
    mLeftLowerLegPos = mLeftUpperLegPos + rp3d::Vector3(0, -3, 0);
    mLeftLowerLegObject = CreateBonePhysics(mLeftLowerLegPos,
                                            rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                            {0.2, 3, 0.2}, 8, "leftLowerLeg");

    mLeftLowerLegBone = CreateBone("leftLowerLeg", mLeftLowerLegObject, mLeftUpperLegBone);
    // --------------- Create the right upper arm Cone --------------- //
    mRightUpperArmPos = mChestPos + rp3d::Vector3(-2.25, 0, 0);
    mRightUpperArmObject = CreateBonePhysics(mRightUpperArmPos,
                                             rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0),
                                             {0.2, 2, 0.2}, 8, "rightUpperArm");

    mRightUpperArmBone = CreateBone("rightUpperArm", mRightUpperArmObject, mChestBone);
    // --------------- Create the right lower arm Cone --------------- //
    mRightLowerArmPos = mRightUpperArmPos + rp3d::Vector3(-2.5, 0, 0);
    mRightLowerArmObject = CreateBonePhysics(mRightLowerArmPos,
                                             rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0),
                                             {0.2, 2, 0.2}, 8, "rightLowerArm");

    mRightLowerArmBone = CreateBone("rightLowerArm", mRightLowerArmObject, mRightUpperArmBone);
    // --------------- Create the right upper leg Cone --------------- //
    mRightUpperLegPos = mHipPos + rp3d::Vector3(-0.8, -1.5, 0);
    mRightUpperLegObject = CreateBonePhysics(mRightUpperLegPos,
                                             rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                             {0.2, 2, 0.2}, 8, "rightUpperLeg");

    mRightUpperLegBone = CreateBone("rightUpperArm", mRightUpperLegObject, mHipBone);
    // --------------- Create the right lower leg Cone --------------- //
    mRightLowerLegPos = mRightUpperLegPos + rp3d::Vector3(0, -3, 0);
    mRightLowerLegObject = CreateBonePhysics(mRightLowerLegPos,
                                             rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                             {0.2, 3, 0.2}, 8, "rightLowerLeg");

    mRightLowerLegBone = CreateBone("rightLowerLeg", mRightLowerLegObject, mRightUpperLegBone);
    // --------------- Create the joint between head and chest --------------- //

    // Create the joint info object
    rp3d::RigidBody *body1 = mHeadObject->getRigidBody();
    rp3d::RigidBody *body2 = mChestObject->getRigidBody();
    rp3d::BallAndSocketJointInfo jointInfo1(body1, body2, mHeadPos + rp3d::Vector3(0, -0.75, 0));
    jointInfo1.isCollisionEnabled = false;
    mHeadChestJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo1));
    mHeadChestJoint->setConeLimitHalfAngle(40.0 * rp3d::PI_RP3D / 180.0);
    mHeadChestJoint->enableConeLimit(true);

    // --------------- Create the joint between chest and left upper arm --------------- //

    // Create the joint info object
    body1 = mChestObject->getRigidBody();
    body2 = mLeftUpperArmObject->getRigidBody();
    rp3d::BallAndSocketJointInfo jointInfo2(body1, body2, mLeftUpperArmPos + rp3d::Vector3(-1, 0, 0));
    jointInfo2.isCollisionEnabled = false;
    mChestLeftUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo2));
    mChestLeftUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
    mChestLeftUpperArmJoint->enableConeLimit(true);

    // --------------- Create the joint between left upper arm and left lower arm  --------------- //

    // Create the joint info object
    body1 = mLeftUpperArmObject->getRigidBody();
    body2 = mLeftLowerArmObject->getRigidBody();
    rp3d::Vector3 joint2WorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::Vector3 joint2WorldAxis(0, 0, 1);
    rp3d::HingeJointInfo jointInfo3(body1, body2, joint2WorldAnchor, joint2WorldAxis);
    jointInfo3.isCollisionEnabled = false;
    mLeftUpperLeftLowerArmJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo3));
    mLeftUpperLeftLowerArmJoint->enableLimit(true);
    mLeftUpperLeftLowerArmJoint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
    mLeftUpperLeftLowerArmJoint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);

    // --------------- Create the joint between chest and waist  --------------- //

    // Create the joint info object
    body1 = mChestObject->getRigidBody();
    body2 = mWaistObject->getRigidBody();
    rp3d::Vector3 jointChestWaistWorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::FixedJointInfo jointChestWaistInfo(body1, body2, jointChestWaistWorldAnchor);
    jointChestWaistInfo.isCollisionEnabled = false;
    mChestWaistJoint = dynamic_cast<rp3d::FixedJoint *>(mPhysicsWorld->createJoint(jointChestWaistInfo));

    // --------------- Create the joint between waist and hips  --------------- //

    // Create the joint info object
    body1 = mWaistObject->getRigidBody();
    body2 = mHipObject->getRigidBody();
    rp3d::Vector3 jointWaistHipsWorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::FixedJointInfo jointWaistHipsInfo(body1, body2, jointWaistHipsWorldAnchor);
    jointWaistHipsInfo.isCollisionEnabled = false;
    mWaistHipsJoint = dynamic_cast<rp3d::FixedJoint *>(mPhysicsWorld->createJoint(jointWaistHipsInfo));

    // --------------- Create the joint between hip and left upper leg --------------- //

    // Create the joint info object
    body1 = mHipObject->getRigidBody();
    body2 = mLeftUpperLegObject->getRigidBody();
    rp3d::BallAndSocketJointInfo jointInfo4(body1, body2, mHipPos + rp3d::Vector3(0.8, 0, 0));
    jointInfo4.isCollisionEnabled = false;
    mHipLeftUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo4));
    mHipLeftUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
    mHipLeftUpperLegJoint->enableConeLimit(true);

    // --------------- Create the joint between left upper leg and left lower leg  --------------- //

    // Create the joint info object
    body1 = mLeftUpperLegObject->getRigidBody();
    body2 = mLeftLowerLegObject->getRigidBody();
    rp3d::Vector3 joint5WorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::Vector3 joint5WorldAxis(1, 0, 0);
    const rp3d::decimal joint5MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
    const rp3d::decimal joint5MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
    rp3d::HingeJointInfo jointInfo5(body1, body2, joint5WorldAnchor, joint5WorldAxis, joint5MinAngle, joint5MaxAngle);
    jointInfo5.isCollisionEnabled = false;
    mLeftUpperLeftLowerLegJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo5));

    // --------------- Create the joint between chest and right upper arm --------------- //

    // Create the joint info object
    body1 = mChestObject->getRigidBody();
    body2 = mRightUpperArmObject->getRigidBody();
    rp3d::BallAndSocketJointInfo jointInfo6(body1, body2, mRightUpperArmPos + rp3d::Vector3(1, 0, 0));
    jointInfo6.isCollisionEnabled = false;
    mChestRightUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo6));
    mChestRightUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
    mChestRightUpperArmJoint->enableConeLimit(true);

    // --------------- Create the joint between right upper arm and right lower arm  --------------- //

    // Create the joint info object
    body1 = mRightUpperArmObject->getRigidBody();
    body2 = mRightLowerArmObject->getRigidBody();
    rp3d::Vector3 joint7WorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::Vector3 joint7WorldAxis(0, 0, 1);
    rp3d::HingeJointInfo jointInfo7(body1, body2, joint7WorldAnchor, joint7WorldAxis);
    jointInfo7.isCollisionEnabled = false;
    mRightUpperRightLowerArmJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo7));
    mRightUpperRightLowerArmJoint->enableLimit(true);
    mRightUpperRightLowerArmJoint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
    mRightUpperRightLowerArmJoint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);

    // --------------- Create the joint between hips and right upper leg --------------- //

    // Create the joint info object
    body1 = mHipObject->getRigidBody();
    body2 = mRightUpperLegObject->getRigidBody();
    rp3d::BallAndSocketJointInfo jointInfo8(body1, body2, mHipPos + rp3d::Vector3(-0.8, 0, 0));
    jointInfo8.isCollisionEnabled = false;
    mHipRightUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo8));
    mHipRightUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
    mHipRightUpperLegJoint->enableConeLimit(true);

    // --------------- Create the joint between right upper leg and right lower leg  --------------- //

    // Create the joint info object
    body1 = mRightUpperLegObject->getRigidBody();
    body2 = mRightLowerLegObject->getRigidBody();
    rp3d::Vector3 joint9WorldAnchor =
            (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
    rp3d::Vector3 joint9WorldAxis(1, 0, 0);
    const rp3d::decimal joint9MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
    const rp3d::decimal joint9MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
    rp3d::HingeJointInfo jointInfo9(body1, body2, joint9WorldAnchor, joint9WorldAxis, joint9MinAngle, joint9MaxAngle);
    jointInfo9.isCollisionEnabled = false;
    mRightUpperRightLowerLegJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo9));

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

//void
//Skeleton::SetLeftUpperLeftLowerArmJointRotation(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
//    mLeftLowerArm->setTransform(rp3d::Transform(mLeftUpperArm->getTransform().getPosition(),
//                                                rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ)));
//}
//

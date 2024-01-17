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
    bones[bone_name] = new_bone;

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
    bones[bone_name] = new_bone;

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

        // --------------- Create the hips Sphere --------------- //
        mHipBone = CreateBone("hip", nullptr, ragdollPosition, rp3d::Quaternion::identity(),
                              mHip_radius, 20, rp3d::Quaternion::identity());
        mHipBone->GetPhysicsObject()->getRigidBody()->setType(rp3d::BodyType::STATIC);

        // --------------- Create the hips left Cone --------------- //
        mHipLeftBone = CreateBone("lButtock", mHipBone, defaultPosition,
                                  rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 3.0),
                                  {0.15, 1, 0.15}, 9, "cone_offset.obj",
                                  rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0 - rp3d::PI_RP3D / 3.0));

        // --------------- Create the hips right Cone --------------- //
        mHipRightBone = CreateBone("rButtock", mHipBone, defaultPosition,
                                   rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 3.0),
                                   {0.15, 1, 0.15}, 9, "cone_offset.obj",
                                   rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 3.0 - rp3d::PI_RP3D / 2.0));

        // --------------- Create the waist Cone --------------- //
        mWaistBone = CreateBone("abdomen", mHipBone, defaultPosition, rp3d::Quaternion::identity(),
                                {0.2, 2, 0.2}, 9, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the chest Cone --------------- //
        mChestBone = CreateBone("chest", mWaistBone, defaultPosition, rp3d::Quaternion::identity(),
                                {0.2, 1.5, 0.2}, 9, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the chest left Cone --------------- //
        mChestLeftBone = CreateBone("lCollar", mWaistBone, defaultPosition,
                                    rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 10.0),
                                    {0.2, 1.5, 0.2}, 9, "cone_offset.obj",
                                    rp3d::Quaternion::fromEulerAngles(0, 0,
                                                                      rp3d::PI_RP3D / 2.0 - rp3d::PI_RP3D / 10.0));

        // --------------- Create the chest right Cone --------------- //
        mChestRightBone = CreateBone("rCollar", mWaistBone, defaultPosition,
                                     rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 10.0),
                                     {0.2, 1.5, 0.2}, 9, "cone_offset.obj",
                                     rp3d::Quaternion::fromEulerAngles(0, 0,
                                                                       rp3d::PI_RP3D / 10.0 + rp3d::PI_RP3D / 2.0));

        // --------------- Create the neck Cone --------------- //
        mNeckBone = CreateBone("neck", mChestBone, defaultPosition, rp3d::Quaternion::identity(),
                               {0.2, 0.8, 0.2}, 9, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the head Sphere --------------- //
        mHeadBone = CreateBone("head", mNeckBone, defaultPosition, rp3d::Quaternion::identity(), 0.75f, 7,
                               rp3d::Quaternion::identity());

        // --------------- Create the left shoulder Cone --------------- //
        mLeftShoulderBone = CreateBone("lCollar", mChestLeftBone, defaultPosition,
                                       rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 1.8),
                                       {0.15, 1, 0.15}, 8, "cone_offset.obj",
                                       rp3d::Quaternion::fromEulerAngles(0, 0,
                                                                         rp3d::PI_RP3D / 2.0 - rp3d::PI_RP3D / 1.8));

        // --------------- Create the left upper arm Cone --------------- //
        mLeftUpperArmBone = CreateBone("lShldr", mLeftShoulderBone, defaultPosition,
                                       rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 2.0),
                                       {0.2, 2, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the left lower arm Cone --------------- //
        mLeftLowerArmBone = CreateBone("lForeArm", mLeftUpperArmBone, defaultPosition,
                                       rp3d::Quaternion::fromEulerAngles(0, 0, -rp3d::PI_RP3D / 2.0),
                                       {0.2, 2, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the left upper leg Cone --------------- //
        mLeftUpperLegBone = CreateBone("lThigh", mHipLeftBone, defaultPosition,
                                       rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                       {0.2, 3, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the left lower leg Cone --------------- //
        mLeftLowerLegBone = CreateBone("lShin", mLeftUpperLegBone, defaultPosition,
                                       rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                       {0.2, 3.5, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the right shoulder Cone --------------- //
        mRightShoulderBone = CreateBone("rCollar", mChestRightBone, defaultPosition,
                                        rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 1.8),
                                        {0.15, 1, 0.15}, 8, "cone_offset.obj",
                                        rp3d::Quaternion::fromEulerAngles(0, 0,
                                                                          rp3d::PI_RP3D / 1.8 - rp3d::PI_RP3D / 2.0));

        // --------------- Create the right upper arm Cone --------------- //
        mRightUpperArmBone = CreateBone("rShldr", mRightShoulderBone, defaultPosition,
                                        rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0),
                                        {0.2, 2, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the right lower arm Cone --------------- //
        mRightLowerArmBone = CreateBone("rForeArm", mRightUpperArmBone, defaultPosition,
                                        rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0),
                                        {0.2, 2, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the right upper leg Cone --------------- //
        mRightUpperLegBone = CreateBone("rThigh", mHipRightBone, defaultPosition,
                                        rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                        {0.2, 3, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        // --------------- Create the right lower leg Cone --------------- //
        mRightLowerLegBone = CreateBone("rShin", mRightUpperLegBone, defaultPosition,
                                        rp3d::Quaternion::fromEulerAngles(rp3d::PI_RP3D, 0, 0),
                                        {0.2, 3.5, 0.2}, 8, "cone_offset.obj", rp3d::Quaternion::identity());

        mHipBone->UpdateChild(rp3d::Quaternion::identity());
        // --------------- Create the joint between head and chest --------------- //

        // Create the joint info object
        rp3d::RigidBody *body1 = mHeadBone->GetPhysicsObject()->getRigidBody();
        rp3d::RigidBody *body2 = mChestBone->GetPhysicsObject()->getRigidBody();
        rp3d::BallAndSocketJointInfo jointInfo1(body1, body2,
                                                mHipBone->GetPosition() + rp3d::Vector3(0, -0.75, 0));
        jointInfo1.isCollisionEnabled = false;
        mHeadChestJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo1));
        mHeadChestJoint->setConeLimitHalfAngle(40.0 * rp3d::PI_RP3D / 180.0);
        mHeadChestJoint->enableConeLimit(true);

        // --------------- Create the joint between chest and left upper arm --------------- //

        // Create the joint info object
        body1 = mChestBone->GetPhysicsObject()->getRigidBody();
        body2 = mLeftUpperArmBone->GetPhysicsObject()->getRigidBody();
        rp3d::BallAndSocketJointInfo jointInfo2(body1, body2,
                                                mLeftUpperArmBone->GetPosition() + rp3d::Vector3(-1, 0, 0));
        jointInfo2.isCollisionEnabled = false;
        mChestLeftUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo2));
        mChestLeftUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
        mChestLeftUpperArmJoint->enableConeLimit(true);

        // --------------- Create the joint between left upper arm and left lower arm  --------------- //

        // Create the joint info object
        body1 = mLeftUpperArmBone->GetPhysicsObject()->getRigidBody();
        body2 = mLeftLowerArmBone->GetPhysicsObject()->getRigidBody();
        rp3d::Vector3 joint2WorldAnchor =
                (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
        rp3d::Vector3 joint2WorldAxis(0, 0, 1);
        rp3d::HingeJointInfo jointInfo3(body1, body2, joint2WorldAnchor,
                                        joint2WorldAxis);
        jointInfo3.isCollisionEnabled = false;
        mLeftUpperLeftLowerArmJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo3));
        mLeftUpperLeftLowerArmJoint->enableLimit(true);
        mLeftUpperLeftLowerArmJoint->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
        mLeftUpperLeftLowerArmJoint->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);

        // --------------- Create the joint between chest and waist  --------------- //

        // Create the joint info object
        body1 = mChestBone->GetPhysicsObject()->getRigidBody();
        body2 = mWaistBone->GetPhysicsObject()->getRigidBody();
        rp3d::Vector3 jointChestWaistWorldAnchor =
                (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
        rp3d::FixedJointInfo jointChestWaistInfo(body1, body2, jointChestWaistWorldAnchor);
        jointChestWaistInfo.isCollisionEnabled = false;
        mChestWaistJoint = dynamic_cast<rp3d::FixedJoint *>(mPhysicsWorld->createJoint(jointChestWaistInfo));

        // --------------- Create the joint between waist and hips  --------------- //

        // Create the joint info object
        body1 = mWaistBone->GetPhysicsObject()->getRigidBody();
        body2 = mHipBone->GetPhysicsObject()->getRigidBody();
        rp3d::Vector3 jointWaistHipsWorldAnchor =
                (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
        rp3d::FixedJointInfo jointWaistHipsInfo(body1, body2, jointWaistHipsWorldAnchor);
        jointWaistHipsInfo.isCollisionEnabled = false;
        mWaistHipsJoint = dynamic_cast<rp3d::FixedJoint *>(mPhysicsWorld->createJoint(jointWaistHipsInfo));

        // --------------- Create the joint between hip and left upper leg --------------- //

        // Create the joint info object
        body1 = mHipBone->GetPhysicsObject()->getRigidBody();
        body2 = mLeftUpperLegBone->GetPhysicsObject()->getRigidBody();
        rp3d::BallAndSocketJointInfo jointInfo4(body1, body2,
                                                mHipBone->GetPosition() + rp3d::Vector3(0.8, 0, 0));
        jointInfo4.isCollisionEnabled = false;
        mHipLeftUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo4));
        mHipLeftUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
        mHipLeftUpperLegJoint->enableConeLimit(true);

        // --------------- Create the joint between left upper leg and left lower leg  --------------- //

        // Create the joint info object
        body1 = mLeftUpperLegBone->GetPhysicsObject()->getRigidBody();
        body2 = mLeftLowerLegBone->GetPhysicsObject()->getRigidBody();
        rp3d::Vector3 joint5WorldAnchor =
                (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
        rp3d::Vector3 joint5WorldAxis(1, 0, 0);
        const rp3d::decimal joint5MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
        const rp3d::decimal joint5MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
        rp3d::HingeJointInfo jointInfo5(body1, body2, joint5WorldAnchor,
                                        joint5WorldAxis, joint5MinAngle, joint5MaxAngle);
        jointInfo5.isCollisionEnabled = false;
        mLeftUpperLeftLowerLegJoint = dynamic_cast<rp3d::HingeJoint *>(mPhysicsWorld->createJoint(jointInfo5));

        // --------------- Create the joint between chest and right upper arm --------------- //

        // Create the joint info object
        body1 = mChestBone->GetPhysicsObject()->getRigidBody();
        body2 = mRightUpperArmBone->GetPhysicsObject()->getRigidBody();
        rp3d::BallAndSocketJointInfo jointInfo6(body1, body2,
                                                mRightUpperArmBone->GetPosition() + rp3d::Vector3(1, 0, 0));
        jointInfo6.isCollisionEnabled = false;
        mChestRightUpperArmJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo6));
        mChestRightUpperArmJoint->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
        mChestRightUpperArmJoint->enableConeLimit(true);

        // --------------- Create the joint between right upper arm and right lower arm  --------------- //

        // Create the joint info object
        body1 = mRightUpperArmBone->GetPhysicsObject()->getRigidBody();
        body2 = mRightLowerArmBone->GetPhysicsObject()->getRigidBody();
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
        body1 = mHipBone->GetPhysicsObject()->getRigidBody();
        body2 = mRightUpperLegBone->GetPhysicsObject()->getRigidBody();
        rp3d::BallAndSocketJointInfo jointInfo8(body1, body2, mHipBone->GetPosition() + rp3d::Vector3(-0.8, 0, 0));
        jointInfo8.isCollisionEnabled = false;
        mHipRightUpperLegJoint = dynamic_cast<rp3d::BallAndSocketJoint *>(mPhysicsWorld->createJoint(jointInfo8));
        mHipRightUpperLegJoint->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
        mHipRightUpperLegJoint->enableConeLimit(true);

        // --------------- Create the joint between right upper leg and right lower leg  --------------- //

        // Create the joint info object
        body1 = mRightUpperLegBone->GetPhysicsObject()->getRigidBody();
        body2 = mRightLowerLegBone->GetPhysicsObject()->getRigidBody();
        rp3d::Vector3 joint9WorldAnchor =
                (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
        rp3d::Vector3 joint9WorldAxis(1, 0, 0);
        const rp3d::decimal joint9MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
        const rp3d::decimal joint9MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
        rp3d::HingeJointInfo jointInfo9(body1, body2, joint9WorldAnchor, joint9WorldAxis, joint9MinAngle,
                                        joint9MaxAngle);
        jointInfo9.isCollisionEnabled = false;
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

void Skeleton::SetJointRotation_bvh(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
    bone->SetJointRotation_bvh(angleX, angleY, angleZ);

    // Event occur!!!
    bone_transform_changed.fire(bone);
    bone->UpdateChild(rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ));
}

void Skeleton::RotateJoint(Bone *bone, rp3d::Vector3 &angle) {
    RotateJoint(bone, angle.x, angle.y, angle.z);
}

void Skeleton::RotateJoint(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ) {
    // fix later ...


//    bone->GetPhysicsObject()->setTransform({bone->GetPosition(), new_quatern});
//
//    // Event occur!!!
//    bone_transform_changed.fire(bone);
//    bone->UpdateChild(rp3d::Quaternion::fromEulerAngles(angleX, angleY, angleZ));
}


Bone *Skeleton::FindBone(rp3d::RigidBody *body) {
    Bone *target = nullptr;
    for (auto &[key, bone]: bones) {
        if (bone->GetPhysicsObject()->getRigidBody() == body) {
            target = bone;
            break;
        }
    }
    return target;
}

Bone *Skeleton::FindBone(const string &name) {
    Bone *target;
    for (auto &[key, bone]: bones) {
        if (key == name) {
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
    rp3d::Vector3 pos;
    rp3d::Vector3 angle;
    for (auto &[name, bone]: bones) {
        if (name == "hip" || name == "rCollar" || name == "lCollar" || name == "rButtock" || name == "lButtock")
            continue;
        pos = bone->GetPosition();

        auto bone_bvh = other_bvh->GetJoint(name);
        for (auto channel: bone_bvh->channels) {
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
        // translate
        auto quatern = bone->GetPhysicsObject()->getTransform().getOrientation();
        bone->GetPhysicsObject()->setTransform({pos, quatern});
        // rotate
        angle = AngleTool::DegreeToEulerAngles(angle);
        SetJointRotation_bvh(bone, angle.x, angle.y, angle.z);

        bone_transform_changed.fire(bone);
    }
}


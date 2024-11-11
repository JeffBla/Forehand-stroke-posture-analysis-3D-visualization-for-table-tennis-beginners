#include "Skeleton.h"
#include "AngleTool.h"

using namespace angleTool;
using namespace skeleton;
using namespace bone;

Skeleton::~Skeleton() {
    delete physics;
    delete motionController;
    delete boneHierarchy;
    delete postureAnalyzer;
}

//void Skeleton::SetJointRotation(const string &boneName, const Vector3 &rotation) {
//    auto bone = boneHierarchy->GetBone(boneName);
//    if (bone) {
//        motionController->GetTransformManager().setWorldRotation(bone, rotation);
//    }
//}

void Skeleton::NextFrame() {
    motionController->ComputerNextFrame();
}

void Skeleton::initializeSkeleton() {
    // Iterate through the BVH joints and create corresponding bones
    for (const auto &joint: bvh->GetJoints()) {
        boneHierarchy->CreateBone(0.15, joint, physics);
    }

    UpdateSkeleton(motionController->InitializeAnimation());
}

void Skeleton::UpdateSkeleton(const map<std::string, rp3d::Transform>&) {
    // Update the physics engine with the new bone transforms
    for (const auto &transform: motionController->GetPendingTransforms()) {
        auto bone = boneHierarchy->GetBone(transform.first);
        if (bone) {
            physics->UpdateBoneTransform(bone->GetPhysicsObject(), transform.second);
            bone_transform_changed.fire(bone);
        }
    }
}


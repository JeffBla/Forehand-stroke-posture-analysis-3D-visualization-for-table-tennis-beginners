/********************************************************************************
 * ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
 * Copyright (c) 2010-2016 Daniel Chappuis                                       *
 *********************************************************************************
 *                                                                               *
 * This software is provided 'as-is', without any express or implied warranty.   *
 * In no event will the authors be held liable for any damages arising from the  *
 * use of this software.                                                         *
 *                                                                               *
 * Permission is granted to anyone to use this software for any purpose,         *
 * including commercial applications, and to alter it and redistribute it        *
 * freely, subject to the following restrictions:                                *
 *                                                                               *
 * 1. The origin of this software must not be misrepresented; you must not claim *
 *    that you wrote the original software. If you use this software in a        *
 *    product, an acknowledgment in the product documentation would be           *
 *    appreciated but is not required.                                           *
 *                                                                               *
 * 2. Altered source versions must be plainly marked as such, and must not be    *
 *    misrepresented as being the original software.                             *
 *                                                                               *
 * 3. This notice may not be removed or altered from any source distribution.    *
 *                                                                               *
 ********************************************************************************/

// Libraries
#include "BvhScene.h"

// Namespaces
using namespace openglframework;
using namespace bvhscene;
using namespace nanogui;
using namespace angleTool;

// Constructor
BvhScene::BvhScene(const std::string &name, EngineSettings &settings, reactphysics3d::PhysicsCommon &physicsCommon)
        : SceneDemo(name, settings, physicsCommon, true, true) {
    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(0.5);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;

    raycastedTarget_bone = nullptr;
}

// Destructor
BvhScene::~BvhScene() {
    destroyPhysicsWorld();
}

// Create the physics world
void BvhScene::createPhysicsWorld() {
    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y,
                                           mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // ------------------------- FLOOR 2 ----------------------- //
    // Create the floor
    mFloor2 = new Box(true, FLOOR_2_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFloor2->setColor(mFloorColorDemo);
    mFloor2->setTransform(rp3d::Transform(rp3d::Vector3(0, -7, 0), rp3d::Quaternion::identity()));
    mFloor2->setSleepingColor(mFloorColorDemo);
    mFloor2->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mFloor2);
}

// Destroy the physics world
void BvhScene::destroyPhysicsWorld() {
    if (mPhysicsWorld != nullptr) {
        delete mFloor2;

        delete skeleton1;

        delete experx_skeleton;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}


void BvhScene::update() {
    if (isMotionStart) {
        double currentTime = glfwGetTime();
        if (lastUpdateTime == 0.0) {
            lastUpdateTime = currentTime;
        } else {
            accumulatedTime += currentTime - lastUpdateTime;
            lastUpdateTime = currentTime;
            if (accumulatedTime > motionInverval) {
                accumulatedTime = 0.0;
                MotionNext();

                motion_nexted.fire();
            }
        }
    }
    SceneDemo::update();
}

// Reset the scene
void BvhScene::reset() {
    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();

    isMotionStart = false;
}

skeleton::Skeleton *BvhScene::CreateSkeleton(string &new_bvh) {
    DestroySkeleton();

    // Create the skeleton with bvh
    bvh = new BVH(new_bvh.c_str());
    skeleton1 = new skeleton::Skeleton(mPhysicsCommon, mPhysicsWorld, mPhysicsObjects, mMeshFolderPath, bvh);
    // Analysizer
    forehand_stroke_analysizer = new analysizer::Analysizer(skeleton1, "forehand_stroke");

    skeleton_created.fire();

    raycastedTarget_bone = skeleton1->FindBone("head");
    raycastedTarget_oldcolor = raycastedTarget_bone->GetPhysicsObject()->getColor();
    RecordRaycastTarget(raycastedTarget_bone);
    return skeleton1;
}

void BvhScene::DestroySkeleton() {
    if (skeleton1 != nullptr) {
        delete skeleton1;
        skeleton1 = nullptr;

        delete bvh;
        bvh = nullptr;

        delete forehand_stroke_analysizer;
        forehand_stroke_analysizer = nullptr;

        raycastedTarget_bone = nullptr;

        motion_nexted.clear();
    }
}

skeleton::Skeleton *BvhScene::CreateExpertSkeleton(string &new_bvh) {
    DestroyExpertSkeleton();

    // Create the skeleton with bvh
    expert_bvh = new BVH(new_bvh.c_str());
    experx_skeleton = new skeleton::Skeleton(mPhysicsCommon, mPhysicsWorld, mPhysicsObjects, mMeshFolderPath,
                                             expert_bvh, rp3d::Vector3(10, 0, 0));

    return experx_skeleton;
}

void BvhScene::DestroyExpertSkeleton() {
    if (experx_skeleton != nullptr) {
        delete experx_skeleton;
        experx_skeleton = nullptr;

        delete expert_bvh;
        expert_bvh = nullptr;
    }
}

skeleton::Skeleton *BvhScene::GetSkeleton() {
    return skeleton1;
}

// Called when a raycast hit occurs (show the information of the angles)
rp3d::decimal BvhScene::notifyRaycastHit(const rp3d::RaycastInfo &raycastInfo) {

    auto *body = dynamic_cast<rp3d::RigidBody *>(raycastInfo.body);

    Bone *target_bone = skeleton1->FindBone(body);
    if (target_bone != nullptr) {
        RecordRaycastTarget(target_bone);
    } else {
        target_bone = experx_skeleton->FindBone(body);
        if (target_bone != nullptr) {
            RecordRaycastTarget(target_bone);
        }
    }

    return SceneDemo::notifyRaycastHit(raycastInfo);
}

void BvhScene::RecordRaycastTarget(Bone *target) {
    raycastedTarget_bone->GetPhysicsObject()->setColor(raycastedTarget_oldcolor);
    raycastedTarget_bone->GetPhysicsObject()->setSleepingColor(raycastedTarget_oldcolor);
    raycastedTarget_bone = target;
    raycastedTarget_oldcolor = raycastedTarget_bone->GetPhysicsObject()->getColor();
    raycastedTarget_bone->GetPhysicsObject()->setColor(pickedColor);
    raycastedTarget_bone->GetPhysicsObject()->setSleepingColor(pickedColor);
    // Event occur!!!
    raycastedTarget_changed.fire(target);

    raycastedTarget_bone_Transform = target->GetPhysicsObject()->getTransform();

#ifdef DEBUG
    cout << raycastedTarget_bone_Transform.getPosition().to_string() << endl;
    rp3d::Vector3 tmp = AngleTool::QuaternionToEulerAngles(raycastedTarget_bone_Transform.getOrientation());
    cout << AngleTool::EulerAnglesToDegree(tmp).to_string() << endl;
#endif
}

bool BvhScene::keyboardEvent(int key, int scancode, int action, int mods) {
    // If the space key has been pressed
    // start bvh motion
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        isMotionStart = !isMotionStart;
        return true;
    }
    if (key == GLFW_KEY_N && action == GLFW_PRESS) {
        MotionNext();

        motion_nexted.fire();
        return true;
    }
    if (key == GLFW_KEY_T && action == GLFW_PRESS) {
        skeleton1->SetJointRotation_local(raycastedTarget_bone, 0, M_PI / 6, 0);
        return true;
    }
//    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
//        ForearmStrokeAnalyze(<#initializer#>);
//        return true;
//    }

    return false;
}

void BvhScene::MotionNext() {
    if (skeleton1 != nullptr)
        skeleton1->NextBvhMotion();
    if (experx_skeleton != nullptr)
        experx_skeleton->NextBvhMotion();
}

void BvhScene::ForearmStrokeAnalyze(const std::string &openposePath) {
    forehand_stroke_analysizer->Analyze(openposePath);
}

string BvhScene::GetForearmStrokeAnalyzeSuggestions() {
    if (forehand_stroke_analysizer != nullptr) {
        return forehand_stroke_analysizer->GetSuggestion();
    }
    return "No skeleton to analyze.";
}
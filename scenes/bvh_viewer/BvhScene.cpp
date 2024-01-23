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
#include "Skeleton.h"
#include "BVH.h"
#include "AngleTool.h"
#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>

#include <cmath>

// Namespaces
using namespace openglframework;
using namespace bvhscene;
using namespace nanogui;
using namespace angleTool;

// Constructor
BvhScene::BvhScene(const std::string &name, EngineSettings &settings, reactphysics3d::PhysicsCommon &physicsCommon)
        : SceneDemo(name, settings, physicsCommon, true, true) {
    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 10, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(2.1);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
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
    mFloor2->setTransform(rp3d::Transform(rp3d::Vector3(0, -10, 0), rp3d::Quaternion::identity()));
    mFloor2->setSleepingColor(mFloorColorDemo);
    mFloor2->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mFloor2);

    // create my skeleton
    bvh = new BVH("out.bvh");
    skeleton1 = new skeleton::Skeleton(mPhysicsCommon, mPhysicsWorld, mPhysicsObjects, mMeshFolderPath, bvh);
    skeleton_created.fire();

    raycastedTarget_bone = skeleton1->FindBone("head");
    raycastedTarget_bone->GetPhysicsObject()->setColor(pickedColor);
    raycastedTarget_bone->GetPhysicsObject()->setSleepingColor(pickedColor);
}

// Destroy the physics world
void BvhScene::destroyPhysicsWorld() {
    if (mPhysicsWorld != nullptr) {
        delete mFloor2;

        delete skeleton1;

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

skeleton::Skeleton *BvhScene::GetSkeleton() {
    return skeleton1;
}

// Called when a raycast hit occurs (show the information of the angles)
rp3d::decimal BvhScene::notifyRaycastHit(const rp3d::RaycastInfo &raycastInfo) {

    auto *body = dynamic_cast<rp3d::RigidBody *>(raycastInfo.body);

    Bone *target_bone = skeleton1->FindBone(body);
    if (target_bone != nullptr) {
        RecordRaycastTarget(target_bone);
    }

    return SceneDemo::notifyRaycastHit(raycastInfo);
}

void BvhScene::RecordRaycastTarget(Bone *target) {
    raycastedTarget_bone->GetPhysicsObject()->setColor(BvhScene::mObjectColorDemo);
    raycastedTarget_bone->GetPhysicsObject()->setSleepingColor(BvhScene::mSleepingColorDemo);
    raycastedTarget_bone = target;
    raycastedTarget_bone->GetPhysicsObject()->setColor(pickedColor);
    raycastedTarget_bone->GetPhysicsObject()->setSleepingColor(pickedColor);
    // Event occur!!!
    raycastedTarget_changed.fire(target);

    raycastedTarget_bone_Transform = target->GetPhysicsObject()->getTransform();
    /// Debug
    cout << raycastedTarget_bone_Transform.getPosition().to_string() << endl;
    rp3d::Vector3 tmp = AngleTool::QuaternionToEulerAngles(raycastedTarget_bone_Transform.getOrientation());
    cout << AngleTool::EulerAnglesToDegree(tmp).to_string() << endl;
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
        return true;
    }
    if (key == GLFW_KEY_T && action == GLFW_PRESS) {
        skeleton1->SetJointRotation_local(raycastedTarget_bone, 0, M_PI / 6, 0);
        return true;
    }

    return false;
}

void BvhScene::MotionNext() {
    skeleton1->NextBvhMotion();
}


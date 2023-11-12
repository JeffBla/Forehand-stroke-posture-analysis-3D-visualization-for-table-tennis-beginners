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

#ifndef BVH_SCENE_H
#define BVH_SCENE_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>

#include "Box.h"
#include "Capsule.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "openglframework.h"
#include "Skeleton.h"
#include "Event.h"

using namespace event;

namespace bvhscene {

// Constants
    const float SCENE_RADIUS = 45.0f;
    const int NB_RAGDOLLS_ROWS = 1;
    const int NB_RAGDOLLS_COLS = 1;
    const int NB_RAGDOLLS = NB_RAGDOLLS_ROWS * NB_RAGDOLLS_COLS;
    const openglframework::Vector3 FLOOR_2_SIZE(60, 0.5f, 82);  // Floor dimensions in meters
    const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
    const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class RagdollScene
    class BvhScene : public SceneDemo {
    protected:
        // -------------------- Attributes -------------------- //

        /** raycasted target & info
         * the default is the head
         */
        /// target
        Bone *raycastedTarget_bone;

        /// info
        rp3d::Transform raycastedTarget_bone_Transform;

    public:
        /// Event for raycasted target
        Event<Bone *> raycastedTarget_changed;

    protected:
        /// Head sphere
        Sphere *mHeadBox[NB_RAGDOLLS];

        /// Chest
        Capsule *mChestCapsule[NB_RAGDOLLS];

        /// Waist capsule
        Capsule *mWaistCapsule[NB_RAGDOLLS];

        /// Hip capsule
        Capsule *mHipCapsule[NB_RAGDOLLS];

        /// Left upper arm capsule
        Capsule *mLeftUpperArmCapsule[NB_RAGDOLLS];

        /// Left lower arm capsule
        Capsule *mLeftLowerArmCapsule[NB_RAGDOLLS];

        /// Left upper leg capsule
        Capsule *mLeftUpperLegCapsule[NB_RAGDOLLS];

        /// Left lower leg capsule
        Capsule *mLeftLowerLegCapsule[NB_RAGDOLLS];

        /// Right upper arm capsule
        Capsule *mRightUpperArmCapsule[NB_RAGDOLLS];

        /// Right lower arm capsule
        Capsule *mRightLowerArmCapsule[NB_RAGDOLLS];

        /// Right upper leg capsule
        Capsule *mRightUpperLegCapsule[NB_RAGDOLLS];

        /// Right lower leg capsule
        Capsule *mRightLowerLegCapsule[NB_RAGDOLLS];

        /// Box for the floor 2
        Box *mFloor2;

        /// Ball-And-Socket joint between head and torso
        rp3d::BallAndSocketJoint *mHeadChestJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and left upper arm
        rp3d::BallAndSocketJoint *mChestLeftUpperArmJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower arm
        rp3d::HingeJoint *mLeftUpperLeftLowerArmJoint[NB_RAGDOLLS];

        /// Fixed joint between chest and waist
        rp3d::FixedJoint *mChestWaistJoint[NB_RAGDOLLS];

        /// Fixed joint between waist and hips
        rp3d::FixedJoint *mWaistHipsJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and left upper leg
        rp3d::BallAndSocketJoint *mHipLeftUpperLegJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower leg
        rp3d::HingeJoint *mLeftUpperLeftLowerLegJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and right upper arm
        rp3d::BallAndSocketJoint *mChestRightUpperArmJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and right lower arm
        rp3d::HingeJoint *mRightUpperRightLowerArmJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and right upper leg
        rp3d::BallAndSocketJoint *mHipRightUpperLegJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower leg
        rp3d::HingeJoint *mRightUpperRightLowerLegJoint[NB_RAGDOLLS];

        rp3d::Vector3 mChestPos[NB_RAGDOLLS];
        rp3d::Vector3 mWaistPos[NB_RAGDOLLS];
        rp3d::Vector3 mHipPos[NB_RAGDOLLS];
        rp3d::Vector3 mHeadPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftUpperArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftLowerArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftUpperLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftLowerLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightUpperArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightLowerArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightUpperLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightLowerLegPos[NB_RAGDOLLS];

        skeleton::Skeleton *skeleton1;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        openglframework::Color pickedColor = openglframework::Color(1.0f, 0.918f, 0.0f, 1.0f);

        // -------------------- Methods -------------------- //

        /// Create the bodies and joints for the ragdoll
        void createRagdolls();

        void RecordRaycastTarget(Bone *target);

    public:
        // -------------------- Methods -------------------- //

        /// Constructor
        BvhScene(const std::string &name, EngineSettings &settings, reactphysics3d::PhysicsCommon &physicsCommon);

        /// Destructor
        virtual ~BvhScene() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// Initialize the bodies positions
        void initBodiesPositions();

        const rp3d::Quaternion &transmitAngleInfo(rp3d::RigidBody *body);

        bool mouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY) override;

        float notifyRaycastHit(const rp3d::RaycastInfo &raycastInfo) override;

        skeleton::Skeleton *GetSkeleton();

        Bone *GetRaycastedTarget_bone() const;
    };

    inline Bone *BvhScene::GetRaycastedTarget_bone() const {
        return raycastedTarget_bone;
    }
}  // namespace bvhscene

#endif

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
#include <cmath>
#include <reactphysics3d/reactphysics3d.h>

#include "Box.h"
#include "Capsule.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "openglframework.h"
#include "Skeleton.h"
#include "Event.h"
#include "Analysizer.h"
#include "VideoToBvhConverter.h"
#include "BVH.h"
#include "AngleTool.h"

using namespace event;

namespace bvhscene {
// Constants
    const float SCENE_RADIUS = 45.0f;
    const openglframework::Vector3 FLOOR_2_SIZE(60, 0.5f, 82);  // Floor dimensions in meters

    class BvhScene : public SceneDemo {
    protected:
        /** raycasted target & info
         * the default is the head
         */
        /// target
        Bone *raycastedTarget_bone;

        /// info
        rp3d::Transform raycastedTarget_bone_Transform;

        skeleton::Skeleton *skeleton1 = nullptr;

        // -------------------- Bvh -------------------- //
        bool isMotionStart;
        double lastUpdateTime = 0.0;
        double accumulatedTime = 0.0;
        double motionInverval = 0.1;
        BVH *bvh;

        // -------------------- Analysizer -------------------- //
        analysizer::Analysizer *forehand_stroke_analysizer;

        // -------------------- Physics -------------------- //
        Box *mFloor2;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        openglframework::Color pickedColor = openglframework::Color(1.0f, 0.918f, 0.0f, 1.0f);

        // -------------------- Methods -------------------- //

        void RecordRaycastTarget(Bone *target);

        void MotionNext();

    public:
        // -------------------- Event -------------------- //
        /// Event for raycasted target
        Event<Bone *> raycastedTarget_changed;

        /// Event for create skeleton
        Event<> skeleton_created;

        Event<> motion_nexted;

        // -------------------- Methods -------------------- //

        /// Constructor
        BvhScene(const std::string &name, EngineSettings &settings, reactphysics3d::PhysicsCommon &physicsCommon);

        /// Destructor
        virtual ~BvhScene() override;

        /// Update the scene
        virtual void update() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        Skeleton *CreateSkeleton(string &new_bvh);

        void DestroySkeleton();

        void ForearmStrokeAnalyze(const std::string &openposePath);

        string GetForearmStrokeAnalyzeSuggestions();

        // -------------------- Events -------------------- //
        float notifyRaycastHit(const rp3d::RaycastInfo &raycastInfo) override;

        /// Called when a keyboard event occurs
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;

        // -------------------- Getter & Setter -------------------- //
        skeleton::Skeleton *GetSkeleton();

        Bone *GetRaycastedTarget_bone() const;

        analysizer::Analysizer *GetForehandStrokeAnalysizer() const;
    };

    inline Bone *BvhScene::GetRaycastedTarget_bone() const {
        return raycastedTarget_bone;
    }

    inline analysizer::Analysizer *BvhScene::GetForehandStrokeAnalysizer() const {
        return forehand_stroke_analysizer;
    }
}  // namespace bvhscene

#endif

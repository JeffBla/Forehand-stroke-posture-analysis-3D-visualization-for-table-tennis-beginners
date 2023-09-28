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

#ifndef SKELETON_H
#define SKELETON_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>

#include "Box.h"
#include "BVH.h"
#include "Sphere.h"
#include "openglframework.h"

namespace skeleton {

// Constants

// Class Skeleton
    class Skeleton {
    private:
        rp3d::PhysicsCommon &mPhysicsCommon;
        rp3d::PhysicsWorld *mPhysicsWorld;
        string &mMeshFolderPath;

        const float linearDamping = 0.02f;
        const float angularDamping = 0.02f;
        const float frictionCoeff = 0.4f;
        const openglframework::Vector3 boxSize;
    protected:
        // -------------------- Attributes -------------------- //

        Box *mHipBox;

        /// Fixed joint between chest and waist
        rp3d::FixedJoint *mChestWaistJoint;

        rp3d::Vector3 mHipPos;

        openglframework::Color objectColor;
        openglframework::Color sleepingColor;

        std::vector<PhysicsObject *> mPhysicsObjects;

        // -------------------- Methods -------------------- //
        Box *CreateBone(const rp3d::Vector3 &pos, const openglframework::Vector3 &size);

        void bvhJointToSkeleton(const rp3d::Vector3 &prevPos, bvh::Joint *joint, float scale);

    public:
        // -------------------- Methods -------------------- //

        /// Constructor
        Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld, std::string &mMeshFolderPath,
                 bvh::Joint *joint);

        /// Destructor
        ~Skeleton();

        /// Initialize the bodies positions
        void initBodiesPositions();
    };

}  // namespace skeleton

#endif

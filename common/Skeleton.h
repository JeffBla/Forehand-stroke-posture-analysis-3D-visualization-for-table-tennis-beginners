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
#include "ConvexMesh.h"

namespace skeleton {

// Constants

// Class Skeleton
    class Skeleton {
    private:
        // -------------------- Attributes -------------------- //

        rp3d::PhysicsCommon &mPhysicsCommon;
        rp3d::PhysicsWorld *mPhysicsWorld;
        string &mMeshFolderPath;
        std::vector<PhysicsObject *> &mPhysicsObjects;

        const float linearDamping = 0.02f;
        const float angularDamping = 0.02f;
        const float frictionCoeff = 0.4f;

        enum BodyTypeGroup {
            HEAD, BODY, LEG, HAND
        };
        vector<PhysicsObject *> headGroup, bodyGroup, legGroup, handGroup;

        // -------------------- Methods -------------------- //

    protected:

        openglframework::Color objectColor = openglframework::Color(0.0f, 0.68f, 0.99f, 1.0f);
        openglframework::Color sleepingColor = openglframework::Color(1.0f, 0.0f, 0.0f, 1.0f);

        rp3d::Vector3 mHipPos;
        Sphere *mHip;

        rp3d::Vector3 mWaistPos;
        ConvexMesh *mWaist;

        rp3d::Vector3 mChestPos;
        ConvexMesh *mChest;

        rp3d::Vector3 mHeadPos;
        Sphere *mHead;

        rp3d::Vector3 mLeftUpperArmPos;
        ConvexMesh *mLeftUpperArm;

        rp3d::Vector3 mLeftLowerArmPos;
        ConvexMesh *mLeftLowerArm;

        rp3d::Vector3 mLeftUpperLegPos;
        ConvexMesh *mLeftUpperLeg;

        rp3d::Vector3 mLeftLowerLegPos;
        ConvexMesh *mLeftLowerLeg;

        rp3d::Vector3 mRightUpperArmPos;
        ConvexMesh *mRightUpperArm;

        rp3d::Vector3 mRightLowerArmPos;
        ConvexMesh *mRightLowerArm;

        rp3d::Vector3 mRightUpperLegPos;
        ConvexMesh *mRightUpperLeg;

        rp3d::Vector3 mRightLowerLegPos;
        ConvexMesh *mRightLowerLeg;

        // Joint
        rp3d::BallAndSocketJoint *mHeadChestJoint;

        rp3d::BallAndSocketJoint *mChestLeftUpperArmJoint;

        rp3d::HingeJoint *mLeftUpperLeftLowerArmJoint;

        rp3d::FixedJoint *mChestWaistJoint;

        rp3d::FixedJoint *mWaistHipsJoint;

        rp3d::BallAndSocketJoint *mHipLeftUpperLegJoint;

        rp3d::HingeJoint *mLeftUpperLeftLowerLegJoint;

        rp3d::BallAndSocketJoint *mChestRightUpperArmJoint;

        rp3d::HingeJoint *mRightUpperRightLowerArmJoint;

        rp3d::BallAndSocketJoint *mHipRightUpperLegJoint;

        rp3d::HingeJoint *mRightUpperRightLowerLegJoint;

        // -------------------- Methods -------------------- //
        void ConfigNewObject(PhysicsObject *new_object, const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation);


        ConvexMesh *
        CreateBone(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, const openglframework::Vector3 &size,
                   rp3d::decimal massDensity,
                   const string &model_file);

        Sphere *
        CreateBoneSphere( const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,float radius, rp3d::decimal massDensity);

        Box *
        CreateBone(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, const openglframework::Vector3 &size,
                   rp3d::decimal massDensity);


    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                 vector<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath,
                 const bvh::Joint *joint);

        /// Destructor
        ~Skeleton();

        /// Initialize the bodies positions
        void initBodiesPositions();
    };

}  // namespace skeleton

#endif

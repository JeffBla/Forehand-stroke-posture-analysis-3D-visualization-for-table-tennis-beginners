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

#include "Bone.h"
#include "Box.h"
#include "BVH.h"
#include "Sphere.h"
#include "openglframework.h"
#include "ConvexMesh.h"

using namespace bone;

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

        // -------------------- Methods -------------------- //

    protected:

        openglframework::Color objectColor = openglframework::Color(0.0f, 0.68f, 0.99f, 1.0f);
        openglframework::Color sleepingColor = openglframework::Color(1.0f, 0.0f, 0.0f, 1.0f);

        rp3d::Vector3 mHipPos;
        Sphere *mHipObject;
        Bone *mHipBone;

        rp3d::Vector3 mWaistPos;
        ConvexMesh *mWaistObject;
        Bone *mWaistBone;

        rp3d::Vector3 mChestPos;
        ConvexMesh *mChestObject;
        Bone *mChestBone;

        rp3d::Vector3 mHeadPos;
        Sphere *mHeadObject;
        Bone *mHeadBone;

        rp3d::Vector3 mLeftUpperArmPos;
        ConvexMesh *mLeftUpperArmObject;
        Bone *mLeftUpperArmBone;

        rp3d::Vector3 mLeftLowerArmPos;
        ConvexMesh *mLeftLowerArmObject;
        Bone *mLeftLowerArmBone;

        rp3d::Vector3 mLeftUpperLegPos;
        ConvexMesh *mLeftUpperLegObject;
        Bone *mLeftUpperLegBone;

        rp3d::Vector3 mLeftLowerLegPos;
        ConvexMesh *mLeftLowerLegObject;
        Bone* mLeftLowerLegBone;

        rp3d::Vector3 mRightUpperArmPos;
        ConvexMesh *mRightUpperArmObject;
        Bone *mRightUpperArmBone;

        rp3d::Vector3 mRightLowerArmPos;
        ConvexMesh *mRightLowerArmObject;
        Bone *mRightLowerArmBone;

        rp3d::Vector3 mRightUpperLegPos;
        ConvexMesh *mRightUpperLegObject;
        Bone *mRightUpperLegBone;

        rp3d::Vector3 mRightLowerLegPos;
        ConvexMesh *mRightLowerLegObject;
        Bone *mRightLowerLegBone;

        bone::Bone *hierarchyBone;

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
        CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                          const openglframework::Vector3 &size, rp3d::decimal massDensity,
                          const string &model_file);

        Sphere *
        CreateBonePhysics_Sphere(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, float radius,
                                 rp3d::decimal massDensity);

        Box *
        CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                          const openglframework::Vector3 &size, rp3d::decimal massDensity);

        Bone *CreateBone(const std::string &bone_name, PhysicsObject *bone_object, Bone *parent);

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

        void SetLeftUpperLeftLowerArmJointRotation(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

        void RotateLeftUpperLeftLowerArmJoint(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);
    };

}  // namespace skeleton

#endif

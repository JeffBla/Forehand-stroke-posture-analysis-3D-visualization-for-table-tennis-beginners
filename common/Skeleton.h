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
#include "Sphere.h"
#include "ConvexMesh.h"
#include "openglframework.h"
#include "BVH.h"
#include "Event.h"

using namespace bvh;
using namespace bone;
using namespace event;

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

        Bone *mHipBone;

        Bone *mHipLeftBone;

        Bone *mHipRightBone;

        Bone *mWaistBone;

        Bone *mChestBone;

        Bone *mChestLeftBone;

        Bone *mChestRightBone;

        Bone *mHeadBone;

        Bone *mNeckBone;

        Bone *mLeftShoulderBone;

        Bone *mLeftUpperArmBone;

        Bone *mLeftLowerArmBone;

        Bone *mLeftUpperLegBone;

        Bone *mLeftLowerLegBone;

        Bone *mRightShoulderBone;

        Bone *mRightUpperArmBone;

        Bone *mRightLowerArmBone;

        Bone *mRightUpperLegBone;

        Bone *mRightLowerLegBone;

        std::map<std::string, Bone *> bones;

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

        BVH *bvh;
        int bvh_frame;

        // -------------------- Methods -------------------- //
        void ConfigNewObject(PhysicsObject *new_object, const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation);


        ConvexMesh *
        CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                          const openglframework::Vector3 &size, rp3d::decimal massDensity,
                          const string &model_file);

        Sphere *CreateBonePhysics(const rp3d::Vector3 &pos, const rp3d::Quaternion &orientation, float radius,
                                  reactphysics3d::decimal massDensity);


        /// Create a Bone with ConvexMesh
        Bone *CreateBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                         const openglframework::Vector3 &size, rp3d::decimal massDensity, const string &model_file,
                         const rp3d::Quaternion &local_coordinate_quatern);

        /// Create a Bone with Sphere shape
        Bone *CreateBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                         float radius, rp3d::decimal massDensity, const rp3d::Quaternion &local_coordinate_quatern);


    public:
        // -------------------- Attributes -------------------- //
        Event<Bone *> bone_transform_changed;

        // -------------------- Methods -------------------- //

        /// Constructor
        Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                 vector<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath, BVH *bvh);

        /// Destructor
        ~Skeleton();

        /** SetJointRotation
         * @details rotate worldly & use Euler angle
         * @param bone
         * @param angle
         */
        void SetJointRotation(Bone *bone, rp3d::Vector3 &angle);

        void SetJointRotation(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

	    /** SetJointRotation_local
         * @details rotate locally & use Euler angle
         * @param bone
         * @param angle
         */
        void SetJointRotation_local(Bone *bone, rp3d::Vector3 &angle);

        void SetJointRotation_local(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

        void SetJointRotation_bvh(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                  const bvh::Joint *bone_bvh);

        void RotateJoint(Bone *bone, rp3d::Vector3 &angle);

        void RotateJoint(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

        Bone *FindBone(rp3d::RigidBody *body);

        Bone *FindBone(const string &name);

        // -------------------- Motion -------------------- //

        void NextBvhMotion();

        void ApplyBvhMotion(const int frame, BVH *other_bvh);
    };

}  // namespace skeleton

#endif

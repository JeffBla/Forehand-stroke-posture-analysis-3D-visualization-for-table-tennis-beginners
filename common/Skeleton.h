#ifndef SKELETON_H
#define SKELETON_H

#include <reactphysics3d/reactphysics3d.h>
#include <algorithm>
#include <list>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Bone.h"
#include "Box.h"
#include "Sphere.h"
#include "ConvexMesh.h"
#include "openglframework.h"
#include "BVH.h"
#include "Event.h"

#define SCALE 0.1f

using namespace bvh;
using namespace bone;
using namespace event;

namespace skeleton {

// Constants

// Class Skeleton
    class Skeleton {
    private:
        // -------------------- Attributes -------------------- //

        /// Physics
        rp3d::PhysicsCommon &mPhysicsCommon;
        rp3d::PhysicsWorld *mPhysicsWorld;
        string &mMeshFolderPath;
        std::list<PhysicsObject *> &mPhysicsObjects;
        const rp3d::Vector3 default_pos{0, 0, 0};
        rp3d::Vector3 mSkeletonPosition;

        float mHip_radius = 0.2f;
        const float linearDamping = 0.02f;
        const float angularDamping = 0.02f;
        const float frictionCoeff = 0.4f;

        /// BVH
        const std::vector<string> target_bone_names{"hip", "abdomen", "chest", "neck", "neck1", "head", "rCollar",
                                                    "rShldr", "rForeArm", "rHand", "lCollar", "lShldr", "lForeArm",
                                                    "lHand", "rButtock", "rThigh", "rShin", "rFoot", "lButtock",
                                                    "lThigh", "lShin", "lFoot"};
    protected:

        openglframework::Color objectColor = openglframework::Color(0.0f, 0.68f, 0.99f, 1.0f);
        openglframework::Color sleepingColor = openglframework::Color(1.0f, 0.0f, 0.0f, 1.0f);

        std::map<const string, Bone *> bones;

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
                         const rp3d::Quaternion &local_coordinate_quatern, const Joint *joint);

        /// Create a Bone with Sphere shape
        Bone *CreateBone(const string &bone_name, Bone *parent, rp3d::Vector3 &pos, const rp3d::Quaternion &orientation,
                         float radius, rp3d::decimal massDensity, const rp3d::Quaternion &local_coordinate_quatern,
                         const Joint *joint);


    public:
        // -------------------- Attributes -------------------- //
        Event<Bone *> bone_transform_changed;

        // -------------------- Methods -------------------- //
        /// Constructor
        Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                 list<PhysicsObject *> &mPhysicsObjects, std::string &mMeshFolderPath, BVH *bvh);

        Skeleton(rp3d::PhysicsCommon &mPhysicsCommon, rp3d::PhysicsWorld *mPhysicsWorld,
                 list<PhysicsObject *> &mPhysicsObjects, string &mMeshFolderPath, BVH *bvh, const rp3d::Vector3 &pos);

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

        void RotateJoint(Bone *bone, rp3d::Vector3 &angle);

        void RotateJoint(Bone *bone, rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

        Bone *FindBone(rp3d::RigidBody *body);

        Bone *FindBone(const string &target_name);

        // -------------------- Motion -------------------- //
        void NextBvhMotion();

        void ApplyBvhMotion(const int frame);

        void InitBvhMotion();

        // -------------------- Getter & Setter -------------------- //
        const BVH *GetBvh() const;

        const std::vector<std::string> &GetTargetBoneNames() const;
    };

    inline const std::vector<std::string> &Skeleton::GetTargetBoneNames() const {
        return target_bone_names;
    }

    inline const BVH *Skeleton::GetBvh() const {
        return bvh;
    }

}  // namespace skeleton

#endif

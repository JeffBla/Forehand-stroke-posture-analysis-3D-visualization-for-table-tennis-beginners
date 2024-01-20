#ifndef TESTBED_BONE_H
#define TESTBED_BONE_H


#include <string>
#include <map>
#include <vector>
#include <queue>

#include "BVH.h"
#include "PhysicsObject.h"

using namespace bvh;

namespace bone {

    enum BoneType {
        CONE, SPHERE
    };

    class Bone {
    private:
        inline static const rp3d::Vector3 default_orientation{0, 1, 0};

        BoneType boneType;
        const std::string bone_name;
        PhysicsObject *bone_object;
        rp3d::Vector3 position;
        rp3d::Vector3 local_angle = {0, 0, 0};
        const rp3d::Quaternion init_quatern;
        rp3d::Quaternion origin_quatern;
        const rp3d::Quaternion init_local_coordinate_quatern;
        rp3d::Quaternion local_coordinate_quatern;
        Bone *parent;
        std::map<std::string, Bone *> children;
        const Joint* bvh_joint = nullptr;

        // -------------------- Methods -------------------- //

        rp3d::Quaternion
        _SetJointRotation_bvh(rp3d::decimal angleX, rp3d::decimal angleY,
                              rp3d::decimal angleZ,
                              const bvh::Joint *bvh_joint);

    public:
        Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
             Bone *parent, rp3d::Quaternion &quatern, rp3d::Quaternion local_coordinate_quatern);

        Bone(const std::string &bone_name, PhysicsObject *bone_object, BoneType boneType, rp3d::Vector3 &pos,
             Bone *parent, const rp3d::Quaternion &quatern, rp3d::Quaternion local_coordinate_quatern);

        ~Bone();

        void AppendChild(Bone *child);

        void SetJointRotation_local(rp3d::Vector3 &angle);

        void SetJointRotation_local( rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ);

        void SetJointRotation_bvh(rp3d::Vector3 &angle, const bvh::Joint *bvh_joint);

        void SetJointRotation_bvh(rp3d::decimal angleX, rp3d::decimal angleY, rp3d::decimal angleZ,
                                  const bvh::Joint *bvh_joint);

        void UpdateChild(const rp3d::Quaternion &changedQuatern);

        std::map<std::string, float> GetAngleWithNeighbor();

        const std::string &GetBoneName();

        PhysicsObject *GetPhysicsObject();

        rp3d::Vector3 &GetPosition();

        rp3d::Vector3 &GetLocalAngle();

        const rp3d::Quaternion &GetInitQuaternion();

        const rp3d::Quaternion &GetOriginQuaternion();

        void SetOriginQuaternion(const rp3d::Quaternion &q);

        const rp3d::Quaternion &GetInitLocalCoordinateQuatern();

        const rp3d::Quaternion &GetLocalCoordinateQuatern();

        void SetLocalCoordinateQuatern(const rp3d::Quaternion &q);

        void SetPosition(rp3d::Vector3 &pos);

        void SetLocalAngle(rp3d::Vector3 &angle);

        BoneType GetBoneType() const;
    };

    inline void Bone::AppendChild(Bone *child) {
        children[child->bone_name] = child;
    };

    inline const std::string &Bone::GetBoneName() {
        return bone_name;
    }

    inline PhysicsObject *Bone::GetPhysicsObject() {
        return bone_object;
    };

    inline rp3d::Vector3 &Bone::GetPosition() {
        return position;
    }

    inline void Bone::SetPosition(rp3d::Vector3 &pos) {
        position = pos;
    }

    inline const rp3d::Quaternion &Bone::GetInitQuaternion() {
        return init_quatern;
    }

    inline const rp3d::Quaternion &Bone::GetOriginQuaternion() {
        return origin_quatern;
    }

    inline void Bone::SetOriginQuaternion(const rp3d::Quaternion &q) {
        origin_quatern = q;
    }

    inline const rp3d::Quaternion &Bone::GetInitLocalCoordinateQuatern(){
        return init_local_coordinate_quatern;
    }

    inline const rp3d::Quaternion &Bone::GetLocalCoordinateQuatern() {
        return local_coordinate_quatern;
    }

    inline void Bone::SetLocalCoordinateQuatern(const rp3d::Quaternion &q) {
        local_coordinate_quatern = q;
    }


    inline BoneType Bone::GetBoneType() const {
        return boneType;
    }

    inline rp3d::Vector3 &Bone::GetLocalAngle() {
        return local_angle;
    }

    inline void Bone::SetLocalAngle(rp3d::Vector3 &angle) {
        angle = local_angle;
    }


}


#endif //TESTBED_BONE_H

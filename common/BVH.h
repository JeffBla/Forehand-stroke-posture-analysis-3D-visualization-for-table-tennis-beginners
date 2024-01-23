#ifndef  _BVH_H_
#define  _BVH_H_

#include <vector>
#include <map>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;

namespace bvh {

    enum ChannelEnum {
        X_ROTATION, Y_ROTATION, Z_ROTATION,
        X_POSITION, Y_POSITION, Z_POSITION
    };
    struct Joint;

    struct Channel {
        Joint *joint;

        ChannelEnum type;

        int index;
    };

    struct Joint {
        string name;
        int index;

        std::vector<Joint *> parents;
        vector<Joint *> children;

        double offset[3];

        bool has_site;
        double site[3];

        vector<Channel *> channels;
    };

    class BVH {

    private:
        bool is_load_success;

        string file_name;
        string motion_name;

        int num_channel;
        vector<Channel *> channels;
        vector<Joint *> joints;
        map<string, Joint *> joint_index;
        vector<vector<ChannelEnum>> rotationOrder;

        int num_frame;
        double interval;
        double *motion;
        float position_scale = 0.1f;
        int current_frame;
        vector<glm::vec3> current_frame_positions;
        vector<glm::vec3> current_frame_angles;

    public:
        BVH();

        explicit BVH(const char *bvh_file_name);

        ~BVH();

        void Clear();

        void Load(const char *bvh_file_name);

        void SetCurrentFrame(int frame);

        // -------------------- Setter & Getter -------------------- //

        bool IsLoadSuccess() const;

        const string &GetFileName() const;

        const string &GetMotionName() const;

        const int GetNumJoint() const;

        const Joint *GetJoint(int no) const;

        const int GetNumChannel() const;

        const Channel *GetChannel(int no) const;

        const Joint *GetJoint(const string &j) const;

        vector<Joint *> GetJoints() const;

        const Joint *GetJoint(const char *j) const;

        int GetNumFrame() const;

        double GetInterval() const;

        double GetMotion(int f, int c) const;

        void SetMotion(int f, int c, double v);

        const vector<ChannelEnum> &GetRotationOrder(int index);

        const vector<glm::vec3> &GetCurrentFramePositions();

        const vector<glm::vec3> &GetCurrentFrameAngles();

        float GetPositionScale() const ;

        void SetPositionScale(float positionScale);
    };

    inline bool BVH::IsLoadSuccess() const { return is_load_success; }

    inline const string &BVH::GetFileName() const { return file_name; }

    inline const string &BVH::GetMotionName() const { return motion_name; }

    inline const int BVH::GetNumJoint() const { return joints.size(); }

    inline const Joint *BVH::GetJoint(int no) const { return joints[no]; }

    inline const int BVH::GetNumChannel() const { return channels.size(); }

    inline const Channel *BVH::GetChannel(int no) const { return channels[no]; }

    inline const Joint *BVH::GetJoint(const string &j) const {
        auto i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : NULL;
    }

    inline vector<Joint *> BVH::GetJoints() const { return joints; }

    inline const Joint *BVH::GetJoint(const char *j) const {
        auto i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : NULL;
    }

    inline int BVH::GetNumFrame() const { return num_frame; }

    inline double BVH::GetInterval() const { return interval; }

    inline double BVH::GetMotion(int f, int c) const { return motion[f * num_channel + c]; }

    inline void BVH::SetMotion(int f, int c, double v) { motion[f * num_channel + c] = v; }

    inline const vector<ChannelEnum> &BVH::GetRotationOrder(int index) {
        return rotationOrder[index];
    }

    inline const vector<glm::vec3> &BVH::GetCurrentFramePositions() {
        return current_frame_positions;
    }

    inline const vector<glm::vec3> &BVH::GetCurrentFrameAngles(){
        return current_frame_angles;
    }

    inline float BVH::GetPositionScale() const {
        return position_scale;
    }

    inline void BVH::SetPositionScale(float positionScale) {
        position_scale = positionScale;
    }
} // namespace bvh

#endif // _BVH_H_

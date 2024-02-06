#ifndef TESTBED_VIDEOTOBVHCONVERTER_H
#define TESTBED_VIDEOTOBVHCONVERTER_H

#include <iostream>
#include <string>

namespace videoToBvhConverter {

    class VideoToBvhConverter {
    private:
        std::string MocapNETPath = "/home/jeffbla/SoftWare/MocapNET4";

        std::string videoPath;
        std::string bvhPath;

        // ----------------- Methods -----------------
        static void sig_handler(int sig);
    public:
        VideoToBvhConverter() = default;

        VideoToBvhConverter(const std::string &videoPath, const std::string &bvhPath);

        ~VideoToBvhConverter();

        void Convert();

        void Convert(const std::string &videoPath, const std::string &bvhPath);

        // ----------------- Getter & Setter -----------------
        void SetMocapNETPath(const std::string &path);

        std::string &GetMocapNETPath();

        void SetVideoPath(const std::string &path);

        std::string &GetVideoPath();

        void SetBvhPath(const std::string &path);

        std::string &GetBvhPath();
    };

    inline void VideoToBvhConverter::SetMocapNETPath(const std::string &path) {
        MocapNETPath = path;
    }

    inline std::string &VideoToBvhConverter::GetMocapNETPath() {
        return MocapNETPath;
    }

    inline void VideoToBvhConverter::SetVideoPath(const std::string &path) {
        videoPath = path;
    }

    inline std::string &VideoToBvhConverter::GetVideoPath() {
        return videoPath;
    }

    inline void VideoToBvhConverter::SetBvhPath(const std::string &path) {
        bvhPath = path;
    }

    inline std::string &VideoToBvhConverter::GetBvhPath() {
        return bvhPath;
    }
}


#endif //TESTBED_VIDEOTOBVHCONVERTER_H

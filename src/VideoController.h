#ifndef TESTBED_VIDEOCONTROLLER_H
#define TESTBED_VIDEOCONTROLLER_H

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <nanogui/common.h>

#include "BVH.h"

namespace videoLoader {

    class VideoController {
    private:
        // -------------------- Constants -------------------- //
        float video_scale = 0.4f;

        // -------------------- Attributes -------------------- //
        std::string videoPath;
        int imageWidth, imageHeight;
        cv::VideoCapture *pVideoCapture;
        int currentFrameIdx;
        /**
         * According to https://stackoverflow.com/questions/31472155/python-opencv-cv2-cv-cv-cap-prop-frame-count-get-wrong-numbers
         * The number of frames in a video is not always accurate.
         * So we need to set the number of frames by looping all of them.
         * */
        int init_nFrame;
        std::vector<std::vector<uchar>> frames;
        nanogui::ImageView *imageView;
        nanogui::Array<int32_t, 2> imgDisplaySize;
        bvh::BVH *targetBVH;

        // Use in extend or shorten frames. Sum the reminder of the factor of frames.
        // So we won't lose frames without considering numbers after floating point.
        static float factor_reminder_sum;

        // -------------------- Methods -------------------- //
        void MatchFrame(int target_nFrame, cv::VideoCapture *pVideoCapture);

        void ExtendFrames(int frameIdx, float frame_factor, cv::Mat &flat_frame);

        void ShortenFrames(int frameIdx, float frame_factor, cv::Mat &flat_frame);

    public:
        VideoController() = default;

        VideoController(const std::string &videoPath, nanogui::ImageView *imageView, bvh::BVH *targetBVH);

        ~VideoController();

        void Load(int num_frame);

        void Load(const std::string &videoPath, int num_fame);

        void Show();

        void Next();

        void Previous();

        // ----------------- Getter & Setter -----------------
        void SetVideoPath(const std::string &path);

        std::string &GetVideoPath();

        cv::VideoCapture &GetVideoCapture();

        void SetImageView(nanogui::ImageView *imageView);

        void SetTargetBVH(bvh::BVH *targetBVH);

        void SetVideoScale(float video_scale);

        float GetVideoScale();
    };

    inline void VideoController::SetVideoPath(const std::string &path) {
        videoPath = path;
    }

    inline std::string &VideoController::GetVideoPath() {
        return videoPath;
    }

    inline void VideoController::SetImageView(nanogui::ImageView *imageView) {
        this->imageView = imageView;
    }

    inline void VideoController::SetTargetBVH(bvh::BVH *targetBVH) {
        this->targetBVH = targetBVH;
    }

    inline void VideoController::SetVideoScale(float video_scale) {
        this->video_scale = video_scale;
    }

    inline float VideoController::GetVideoScale() {
        return video_scale;
    }
}
#endif //TESTBED_VIDEOCONTROLLER_H

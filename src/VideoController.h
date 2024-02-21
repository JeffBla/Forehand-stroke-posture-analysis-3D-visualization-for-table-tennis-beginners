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

#define SCALE 0.4

namespace videoLoader {

    class VideoController {
    private:
        std::string videoPath;
        int imageWidth, imageHeight;
        cv::VideoCapture *pVideoCapture;
        int currentFrameIdx;
        std::vector<std::vector<uchar>> frames;
        nanogui::ImageView *imageView;
        nanogui::Array<int32_t, 2> imgDisplaySize;
    public:
        VideoController() = default;

        VideoController(const std::string &videoPath, nanogui::ImageView *imageView);

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
}
#endif //TESTBED_VIDEOCONTROLLER_H

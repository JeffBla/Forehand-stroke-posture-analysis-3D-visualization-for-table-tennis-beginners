#include "VideoController.h"

using namespace videoLoader;

VideoController::VideoController(const std::string &videoPath, nanogui::ImageView *imageView)
        : videoPath(videoPath), pVideoCapture(nullptr),
          currentFrameIdx(0), imageView(imageView) {}

VideoController::~VideoController() {
}

void VideoController::Load(int num_frame) {
    pVideoCapture = new cv::VideoCapture(videoPath);
    if (!pVideoCapture->isOpened()) {
        throw std::runtime_error("VideoController::Load: Video not found");
    }

    cv::Mat frame, resized_frame;
    // Set up some info
    *pVideoCapture >> frame;
    imageWidth = frame.cols;
    imageHeight = frame.rows;

    imgDisplaySize = {(int32_t) (imageWidth * SCALE), (int32_t) (imageHeight * SCALE)};

    currentFrameIdx = 0;
    frames.clear();
    while (pVideoCapture->read(frame) && frames.size() < num_frame) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        cv::resize(frame, resized_frame, cv::Size(imgDisplaySize.x(), imgDisplaySize.y()), 0, 0,
        cv::INTER_LINEAR);

        // Mat to Texture (vector<uchar>)
        uint total_elements = resized_frame.total() * resized_frame.elemSize();
        cv::Mat flat = resized_frame.clone().reshape(1, total_elements);

        frames.emplace_back(flat.data, flat.data + flat.total());
    }

    while(frames.size() != num_frame) {
        frames.push_back(frames.back());
    }

    delete pVideoCapture;

    Show();
}

void VideoController::Load(const std::string &videoPath, int num_fame) {
    this->videoPath = videoPath;
    Load(num_fame);
}

void VideoController::Show() {
    auto old_image = imageView->image();
    if (old_image == nullptr) {
        old_image = new nanogui::Texture(nanogui::Texture::PixelFormat::BGR, nanogui::Texture::ComponentFormat::UInt8,
                                         imgDisplaySize, nanogui::Texture::InterpolationMode::Nearest,
                                         nanogui::Texture::InterpolationMode::Nearest);
        imageView->set_width(imgDisplaySize.x());
        imageView->set_height(imgDisplaySize.y());
        imageView->set_image(old_image);
    }
    old_image->upload(frames[currentFrameIdx].data());
}

void VideoController::Next() {
    currentFrameIdx = (currentFrameIdx + 1) % frames.size();

    Show();
}

void VideoController::Previous() {
    currentFrameIdx = (currentFrameIdx - 1 + frames.size()) % frames.size();

    Show();
}


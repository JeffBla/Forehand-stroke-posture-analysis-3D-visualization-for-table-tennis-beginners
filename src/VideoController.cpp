#include "VideoController.h"

using namespace videoLoader;

float VideoController::factor_reminder_sum = 0;

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

    currentFrameIdx = 0;
    frames.clear();

    MatchFrame(num_frame, pVideoCapture);

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

void VideoController::MatchFrame(int target_nFrame, cv::VideoCapture *pVideoCapture) {
    if (!pVideoCapture->isOpened()) {
        throw std::runtime_error("VideoController::MatchFrame: Video not found");
    }

    cv::Mat frame, resized_frame;
    // Set up some info
    *pVideoCapture >> frame;
    imageWidth = frame.cols;
    imageHeight = frame.rows;

    imgDisplaySize = {(int32_t) (imageWidth * video_scale), (int32_t) (imageHeight * video_scale)};

    int init_nFrame = pVideoCapture->get(cv::CAP_PROP_FRAME_COUNT);
    float frame_factor = target_nFrame / static_cast<float> (init_nFrame);

    factor_reminder_sum = 0;
    frames.clear();
    bool is_read_success;
    cv::Mat flat;
    while (frames.size() < target_nFrame) {
        is_read_success = pVideoCapture->read(frame);
        if (!is_read_success) {
            if (frames.empty()) {
                throw std::runtime_error("VideoController::MatchFrame: Video frame is empty");
            } else {
                for (int i = 0; i < target_nFrame - frames.size(); i++)
                    frames.emplace_back(flat.data, flat.data + flat.total());
                break;
            }
        }

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        cv::resize(frame, resized_frame, cv::Size(imgDisplaySize.x(), imgDisplaySize.y()), 0, 0,
                   cv::INTER_LINEAR);

        // Mat to Texture (vector<uchar>)
        uint total_elements = resized_frame.total() * resized_frame.elemSize();
        flat = resized_frame.clone().reshape(1, total_elements);

        if (frame_factor >= 1)
            ExtendFrames(frame_factor, flat);
        else // frame_factor < 1
            ShortenFrames(frame_factor, flat);
    }
}

void VideoController::ExtendFrames(float frame_factor, cv::Mat &flat_frame) {
    int require_nFrame = static_cast<int> (frame_factor);
    factor_reminder_sum += frame_factor - require_nFrame;
    if (factor_reminder_sum >= 1) {
        require_nFrame += 1;
        factor_reminder_sum -= 1;
    }
    for (int i = 0; i < require_nFrame; i++) {
        frames.emplace_back(flat_frame.data, flat_frame.data + flat_frame.total());
    }
}

void VideoController::ShortenFrames(float frame_factor, cv::Mat &flat_frame) {
    factor_reminder_sum += frame_factor;
    if (factor_reminder_sum >= 1) {
        frames.emplace_back(flat_frame.data, flat_frame.data + flat_frame.total());
        factor_reminder_sum -= 1;
    }
}


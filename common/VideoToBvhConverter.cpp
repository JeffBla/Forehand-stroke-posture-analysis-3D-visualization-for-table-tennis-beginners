#include <csignal>
#include <wait.h>
#include "VideoToBvhConverter.h"

using namespace videoToBvhConverter;

VideoToBvhConverter::VideoToBvhConverter(const std::string &videoPath, const std::string &bvhPath) {
    this->videoPath = videoPath;
    this->bvhPath = bvhPath;
}

void VideoToBvhConverter::sig_handler(int sig) {
    int status;

    if (sig == SIGCHLD) {
        wait(&status);

        if (WIFEXITED(status)) {
            std::cout << "Convert video to bvh successfully!" << std::endl;
        } else {
            printf("Conversion failed!\n");
        }
    }
}

void VideoToBvhConverter::Convert() {
    if (!videoPath.empty() && !bvhPath.empty())
        Convert(videoPath, bvhPath);
}

void VideoToBvhConverter::Convert(const std::string &videoPath, const std::string &bvhPath) {
    // In this redirecting way, there will be some security issues like shell injection.
    // E.g. if the videoPath is "a; rm -rf /"
    const std::string logPath = videoPath.substr(0, videoPath.find_last_of('.')) + ".log";
    const std::string logStderrPath = videoPath.substr(0, videoPath.find_last_of('.')) + "_stderr_" + ".log";
    std::string main_command =
            MocapNETPath + "/scripts/" + "dump_and_process_video_2.sh" + " " + videoPath + " " + bvhPath + " > " +
            logPath + " 2> " + logStderrPath;
    // Use fork to asynchronously execute the command
    signal(SIGCHLD, sig_handler);

    pid_t p = fork();
    if (p < 0) {
        perror("VideoToBvhConverter:fork fail");
        exit(1);
    } else if (p == 0) {
        execl("/bin/sh", "sh", "-c", main_command.c_str(), NULL);
        // If execl() fails
        perror("VideoToBvhConverter:execl fail");
        exit(1);
    }

//    auto status = system(main_command.c_str());
}
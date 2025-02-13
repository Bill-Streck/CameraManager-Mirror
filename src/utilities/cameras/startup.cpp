/**
 * @file startup.cpp
 * @author William Streck
 * @brief Initialization procedures for the cameras.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#include "startup.hpp"

cv::VideoCapture create_capture(settings set) {
    std::string dev_file;
    if (set.device_index == -1) {
        // XXX Subject to change on new wrist camera
        dev_file = "/dev/urc/cam/arducam_01";
    } else {
        auto numstr = std::to_string(set.device_index);
        if (numstr.size() == 1) {
            numstr = "0" + numstr;
        }
        dev_file = "/dev/urc/cam/logitech_" + numstr;
    }

    auto cap = cv::VideoCapture(dev_file, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // mandatory
    cap.set(cv::CAP_PROP_FRAME_WIDTH, set.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, set.height);
    cap.set(cv::CAP_PROP_FPS, set.fps);

    // optional
    cap.set(cv::CAP_PROP_BRIGHTNESS, set.brightness);
    cap.set(cv::CAP_PROP_CONTRAST, set.contrast);
    cap.set(cv::CAP_PROP_SATURATION, set.saturation);
    cap.set(cv::CAP_PROP_GAIN, set.gain);
    cap.set(cv::CAP_PROP_AUTO_WB, set.enable_auto_white_balance);

    return cap;
}
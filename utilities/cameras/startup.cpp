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
    auto cap = cv::VideoCapture(set.device_index);

    // mandatory
    cap.set(cv::CAP_PROP_FRAME_WIDTH, set.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, set.height);
    // cap.set(cv::CAP_PROP_FPS, set.fps);

    // optional
    cap.set(cv::CAP_PROP_BRIGHTNESS, set.brightness);
    cap.set(cv::CAP_PROP_CONTRAST, set.contrast);
    cap.set(cv::CAP_PROP_SATURATION, set.saturation);
    cap.set(cv::CAP_PROP_HUE, set.hue);
    cap.set(cv::CAP_PROP_GAIN, set.gain);
    cap.set(cv::CAP_PROP_EXPOSURE, set.exposure);
    cap.set(cv::CAP_PROP_GAMMA, set.gamma);
    cap.set(cv::CAP_PROP_AUTO_WB, set.enable_auto_white_balance);

    return cap;
}
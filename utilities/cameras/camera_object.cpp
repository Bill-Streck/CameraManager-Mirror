/**
 * @file camera_object.cpp
 * @author William Streck
 * @brief Camera object implementation.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#include "camera_object.hpp"

Camera::Camera() {}

int Camera::get_device_index() {
    return set.device_index;
}

void Camera::configure(settings set) {
    if (running) {
        stop_all();
    }
    this->set = set;
    ready_start = true;
    if (running) {
        cap = create_capture(set);
        start();
    }
}

bool Camera::ready() {
    return ready_start;
}

bool Camera::start() {
    if (!ready_start) {
        return false;
    }
    cap = create_capture(set);
    running = true;
    return true;
}

cv::Mat Camera::get_current_frame() {
    if (!running) {
        return cv::Mat(); // empty frame
    }
    cv::Mat frame = cv::Mat();
    cap >> frame;
    return frame;
}

void Camera::stop_all() {
    running = false;
    cap.release();
}

cv::VideoCapture Camera::get_capture() {
    return cap;
}
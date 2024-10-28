/**
 * @file camera_object.cpp
 * @author William Streck
 * @brief Camera object implementation.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#include "camera_object.hpp"

Camera::Camera(int idx) {
    set.device_index = idx;
}

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

// TODO should be an object instead of a pointer
cv::Mat Camera::get_current_frame() {
    if (!running) {
        return cv::Mat(); // empty frame
    }
    // TODO get the current frame
    cv::Mat frame = cv::Mat();
    cap >> frame;
    return frame;
}

void Camera::stop_all() {
    // TODO stop all camera operations
    running = false;
    cap.release();
}

void Camera::hard_reset() {
    // TODO hard reset (replug somehow) the camera
    stop_all();
    ready_start = false;
}

cv::VideoCapture Camera::get_capture() {
    return cap;
}
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

void Camera::configure(settings set) {
    if (running) {
        stop_all();
    }
    this->set = set;
    ready_start = true;
    if (running) {
        cap = create_capture(set);
        start();
        if (streaming) {
            stream();
        }
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

bool Camera::stream() {
    if (!running) {
        return false;
    }
    // TODO stream the camera
    streaming = true;
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
    streaming = false;
    cap.release();
}

void Camera::stop_stream() {
    // TODO stop the camera stream
    streaming = false;
}

void Camera::hard_reset() {
    // TODO hard reset (replug somehow) the camera
    stop_all();
    cap.release();
    ready_start = false;
}

cv::VideoCapture Camera::get_capture() {
    return cap;
}
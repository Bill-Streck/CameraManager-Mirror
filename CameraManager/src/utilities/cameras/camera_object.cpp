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

bool Camera::change_attribute(int attribute, int modifier) {
    if (!running) {
        return false;
    }

    if (attribute == ATTR_BRIGHTNESS) {
        set.brightness += modifier;

        // Keep in bounds - no reason to deny the command
        if (set.brightness < GENERIC_ATTRBUTE_LIMIT_LOW) {
            set.brightness = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (set.brightness > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            set.brightness = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_BRIGHTNESS, set.brightness);
    } else if (attribute == ATTR_CONTRAST) {
        set.brightness += modifier;

        // Keep in bounds - no reason to deny the command
        if (set.contrast < GENERIC_ATTRBUTE_LIMIT_LOW) {
            set.contrast = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (set.contrast > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            set.contrast = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_CONTRAST, set.contrast);
    } else if (attribute == ATTR_SATURATION) {
        set.saturation += modifier;

        // Keep in bounds - no reason to deny the command
        if (set.saturation < GENERIC_ATTRBUTE_LIMIT_LOW) {
            set.saturation = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (set.saturation > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            set.saturation = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_SATURATION, set.saturation);
    } else if (attribute = ATTR_SHARPNESS) {
        set.sharpness += modifier;

        // Keep in bounds - no reason to deny the command
        if (set.sharpness < GENERIC_ATTRBUTE_LIMIT_LOW) {
            set.sharpness = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (set.sharpness > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            set.sharpness = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_SHARPNESS, set.sharpness);
    } else if (attribute == ATTR_GAIN) {
        set.gain += modifier;

        // Keep in bounds - no reason to deny the command
        if (set.gain < GENERIC_ATTRBUTE_LIMIT_LOW) {
            set.gain = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (set.gain > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            set.gain = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        // This may very well return false, that's fine
        return cap.set(cv::CAP_PROP_GAIN, set.gain);
    } else if (attribute == ATTR_AUTO_WHITE_BALANCE) {
        // Pure toggle - ignore modifier
        if (set.enable_auto_white_balance == 0) {
            set.enable_auto_white_balance = 1;
        } else {
            set.enable_auto_white_balance = 0;
        }

        // Should still return true
        return cap.set(cv::CAP_PROP_AUTO_WB, set.enable_auto_white_balance);
    }

    // User did not provide a valid attribute
    return false;
}

cv::VideoCapture Camera::get_capture() {
    return cap;
}
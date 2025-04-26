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
    return sett.device_index;
}

void Camera::configure(settings sett) {
    if (running) {
        stop_all();
    }
    this->sett = sett;
    ready_start = true;
    if (running) {
        cap = create_capture(sett);
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
    cap = create_capture(sett);
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

bool Camera::change_attribute(int attribute, int value) {
    if (!running) {
        return false;
    }

    if (attribute == ATTR_BRIGHTNESS) {
        sett.brightness = value;

        // Keep in bounds - no reason to deny the command
        if (sett.brightness < GENERIC_ATTRBUTE_LIMIT_LOW) {
            sett.brightness = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (sett.brightness > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            sett.brightness = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_BRIGHTNESS, sett.brightness);
    } else if (attribute == ATTR_CONTRAST) {
        sett.brightness = value;

        // Keep in bounds - no reason to deny the command
        if (sett.contrast < GENERIC_ATTRBUTE_LIMIT_LOW) {
            sett.contrast = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (sett.contrast > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            sett.contrast = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_CONTRAST, sett.contrast);
    } else if (attribute == ATTR_SATURATION) {
        sett.saturation = value;

        // Keep in bounds - no reason to deny the command
        if (sett.saturation < GENERIC_ATTRBUTE_LIMIT_LOW) {
            sett.saturation = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (sett.saturation > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            sett.saturation = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_SATURATION, sett.saturation);
    } else if (attribute == ATTR_SHARPNESS) {
        sett.sharpness = value;

        // Keep in bounds - no reason to deny the command
        if (sett.sharpness < GENERIC_ATTRBUTE_LIMIT_LOW) {
            sett.sharpness = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (sett.sharpness > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            sett.sharpness = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        return cap.set(cv::CAP_PROP_SHARPNESS, sett.sharpness);
    } else if (attribute == ATTR_GAIN) {
        sett.gain = value;

        // Keep in bounds - no reason to deny the command
        if (sett.gain < GENERIC_ATTRBUTE_LIMIT_LOW) {
            sett.gain = GENERIC_ATTRBUTE_LIMIT_LOW;
        } else if (sett.gain > GENERIC_ATTRBUTE_LIMIT_HIGH) {
            sett.gain = GENERIC_ATTRBUTE_LIMIT_HIGH;
        }

        // This may very well return false, that's fine
        return cap.set(cv::CAP_PROP_GAIN, sett.gain);
    } else if (attribute == ATTR_AUTO_WHITE_BALANCE) {
        // Pure toggle - ignore value
        if (sett.enable_auto_white_balance == 0) {
            sett.enable_auto_white_balance = 1;
        } else {
            sett.enable_auto_white_balance = 0;
        }

        // Should still return true
        return cap.set(cv::CAP_PROP_AUTO_WB, sett.enable_auto_white_balance);
    } else if (attribute == ATTR_INTERNAL_RES) {
        // We just change the resolution and don't need to restart the capture
        if (lowest <= value && value <= highest) {
            auto placeholder_fps = sett.fps;
            sett.use_preset(static_cast<clarity>(value));
            sett.fps = placeholder_fps;
            
            // Only set the resolution as we saved the fps
            return cap.set(cv::CAP_PROP_FRAME_WIDTH, sett.width) && cap.set(cv::CAP_PROP_FRAME_HEIGHT, sett.height);
        }
    } else if (attribute == ATTR_INTERNAL_FPS) {
        // Change the fps then unfortunately restart the capture
        if (value > 0) {
            // Round the value up to the nearest 5
            auto fps_val = (value % 5 == 0) ? value : (value + (5 - (value % 5)));
            if (fps_val > 30) {
                fps_val = 30;
            }
            sett.fps = fps_val;
            cap.release(); // Deallocates memory as well -> safe to overwrite cap
            cap = create_capture(sett);
            
            // This is a double call but it won't change anything - just want to pull the correct bool
            return cap.set(cv::CAP_PROP_FPS, sett.fps);
        }
        return true;
    }

    // User did not provide a valid attribute
    return false;
}

cv::VideoCapture Camera::get_capture() {
    return cap;
}
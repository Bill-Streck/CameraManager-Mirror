/**
 * @file startup.hpp
 * @author William Streck
 * @brief Initialization procedures for the cameras.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#ifndef STARTUP_HPP
#define STARTUP_HPP

#include "settings.hpp"
#include <opencv4/opencv2/opencv.hpp>

// [ ] Check eventually
#define MAX_CAMERA_ID 19 ///< Maximum camera ID allowed (Based on hardware, TBD long term)

/**
 * @brief Configures the camera with the given settings.
 * 
 * @param set settings object with the desired camera settings.
 * @return cv::VideoCapture the configured camera.
 */
cv::VideoCapture create_capture(settings set);

#endif

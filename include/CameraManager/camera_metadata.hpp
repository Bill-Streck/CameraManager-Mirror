/**
 * @file image_metadata.hpp
 * @author William Streck
 * @brief Contains information for image metadata
 * @version 0.1
 * @date 2025-01-25
 * 
 */

#ifndef CAMERA_METADATA_HPP
#define CAMERA_METADATA_HPP

#include <cstdint>
#include "robot_interfaces/msg/image_metadata.hpp"
#include "startup.hpp"

#define FOCAL_LENGTH_MM 3.67f ///< Focal length in mm of a Logitech C920 webcam
#define SENSOR_HEIGHT_MM 3.0f ///< Sensor height in mm of a Logitech C920 webcame

const float CAMERA_HEIGHTS[MAX_CAMERA_ID+1] = {
    
}; ///< Array of camera mount heights in centimeters - MUST CHECK IF H = 0 -> Not recorded!!

#endif
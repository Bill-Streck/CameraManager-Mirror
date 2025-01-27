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
#include "camera_manager/msg/image_metadata.hpp"
#include "startup.hpp"

const float FOCAL_LENGTHS[MAX_CAMERA_ID+1] = {
    
}; ///< Array of camera focal lengths - MUST CHECK IF L = 0 -> Not recorded!!

#endif
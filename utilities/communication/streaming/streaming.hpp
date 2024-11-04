/**
 * @file streaming.hpp
 * @author William Streck
 * @brief Defines functions for streaming video with ffmpeg.
 * Process utility for streaming - should be called by handlers that can take care of pipe information.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#ifndef STREAMING_H
#define STREAMING_H

#include <string>
#include "utilities/cameras/settings.hpp"

#define AV1_PRESET "8" ///< AV1 preset for ffmpeg.

/**
 * @brief Starts the ffmpeg streaming process for a camera. Consumes the device resource.
 * 
 * @param set Settings object for the camera.
 * @param camera_id Camera device index.
 * @return FILE* pointer to the pipe for the ffmpeg process. Can pclose from here. nullptr on error.
 */
FILE* ffmpeg_stream_camera(settings set, int camera_id);

#endif

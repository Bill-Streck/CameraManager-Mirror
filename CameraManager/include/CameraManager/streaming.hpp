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
#include <tuple>
#include <iostream>
#include "settings.hpp"
#include "startup.hpp"

#define AV1_PRESET "11" ///< AV1 preset for ffmpeg.
#define FIFO_MODE 0666 ///< Mode for IPC FIFO

/**
 * @brief Starts the ffmpeg streaming process for a camera. Consumes the device resource.
 * 
 * @param set Settings object for the camera.
 * @param camera_id Camera device index.
 * @return FILE* pointer to the pipe for the ffmpeg process. Can pclose from here. nullptr on error.
 * @return int File descriptor for the process output FIFO. -1 on error.
 */
std::tuple<FILE*, int> ffmpeg_stream_camera(settings set, int camera_id);

#endif

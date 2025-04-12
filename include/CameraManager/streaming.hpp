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
#include <iostream>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "settings.hpp"
#include "startup.hpp"

#define AV1_PRESET "11" ///< AV1 preset for ffmpeg.
#define FIFO_MODE 0666 ///< Mode for IPC FIFO. 6 -> 4 (read) + 2 (write) + 0 (no execution); x3 -> owner, groups, others.
#define BROADCAST_ENABLED 1 ///< Just represents a true broadcast parameter value

/**
 * @brief Starts the ffmpeg streaming process for a camera. Consumes the device resource.
 * 
 * @param set Settings object for the camera.
 * @param camera_id Camera device index.
 * @return FILE* pointer to the pipe for the ffmpeg process. Can pclose from here. nullptr on error.
 */
FILE* ffmpeg_stream_camera(settings set, int camera_id);

#endif

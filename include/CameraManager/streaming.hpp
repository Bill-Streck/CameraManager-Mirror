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

using namespace std;

#define MEDIA_SERVER_NAME "mediamtx" ///< Name of the media server process. Tested with version v1.11.3
#define MEDIA_SERVER_STARTUP_BUFFER 100 ///< Buffer time in ms for the media server to start. Safer for stream launches.
#define RTSP_TRANSPORT_ENDPOINT "rtsp://localhost:8554/" ///< RTSP transport endpoint. Localhost for now.

/**
 * @brief Starts the ffmpeg streaming process for a camera. Consumes the device resource.
 * 
 * @param set Settings object for the camera.
 * @param camera_id Camera device index.
 * @return FILE* pointer to the pipe for the ffmpeg process. Can pclose from here. nullptr on error.
 */
FILE* ffmpeg_stream_camera(settings set, int camera_id);

#endif

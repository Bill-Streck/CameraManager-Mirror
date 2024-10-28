/**
 * @file streaming.cpp
 * @author William Streck
 * @brief ffmpeg streaming implementation.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "streaming.hpp"
#include <iostream>

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // TODO will eventually need to handle special settings
    // TODO confirm cam idx will match device
    std::string command = "ffmpeg -f v4l2 -pixel_format yuv420p -i /dev/video" + std::to_string(camera_id) + " " +
    "-s " + set.get_resolution_for_ffmpeg() + // TODO get resolution from settings
    "-framerate " + std::to_string(set.fps) + " " +
    "-i -an -c:v libsvtav1 -preset 8 -f mpegts udp://localhost:9999";

    FILE* pipe = popen(command.c_str(), "w");
    if (!pipe) {
        std::cerr << "Error opening pipe for ffmpeg" << std::endl;
        return NULL;
    }
}
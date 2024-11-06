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

std::string pipeline = "appsrc ! videoconvert ! video/x-raw, format=BGR ! av1enc ! rtpav1pay ! udpsink host=127.0.0.1 port=9999";

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    // std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) +
    // " -r " + std::to_string(set.fps) + " -i - -c:v libsvtav1 -preset " + AV1_PRESET +
    // " -f matroska udp://localhost:9999"; // TODO actual outputs (no idea how to choose these yet :) )

    // H264 fallback (5-6x the bitrate, but much lower latency)
    std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) +
    " -r " + std::to_string(set.fps) + " -i - -c:v libx264 -preset veryfast -tune zerolatency" +
    " -f mpegts udp://localhost:9999 > /dev/null 2>&1";

    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        std::cerr << "Error opening pipe for ffmpeg" << std::endl;
        return nullptr;
    }
    return pipe;
}

/**
 * @file streaming.cpp
 * @author William Streck
 * @brief ffmpeg streaming implementation.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "streaming.hpp"
#include <thread>
#include <unistd.h>
#include <fcntl.h>

static std::string stream_name_from_id(int camera_id) {
    if (camera_id == -1) {
        return "wrist";
    } else {
        return "logi" + std::to_string(camera_id);
    }
}

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // H264 ffmpeg string
    std::string command = 
    "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + // ffmpeg, rawvideo format, pixel format BGR-24bit(8 per), size
    std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) + // size string
    " -re" + // Accept frames at rate received (no set frame rate)
    " -i -" + // Pipe input (- short for pipe:0)
    " -c:v libx264 -preset veryfast -tune zerolatency" + // libx264, verfast, zerolatency tune
    // TODO rtsp from mystream to camera identifier
    // TODO bitrate
    " -f rtsp -rtsp_transport tcp rtsp://localhost:8554/" + stream_name_from_id(camera_id) + // RTSP link
    // [ ] silence terminal output so ssh isn't flooded
    " -loglevel quiet"; // Silence terminal output

    // TODO delete test string
    // std::string command = std::string() +
    // "ffmpeg -f v4l2 -i /dev/video2 -c:v libx264 -preset veryfast -tune zerolatency -f "+
    // "rtsp -rtsp_transport tcp rtsp://localhost:8554/mystream";

    // Open pipe as write - we can read with the fifo
    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        // FIXME either this or the loop needs to be retrying if they find nullptr
        // TODO debug message should also be sent
        return nullptr;
    }

    return pipe;
}

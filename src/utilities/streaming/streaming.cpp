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

/**
 * @brief Returns the stream name (route) from the camera ID.
 * 
 * @param camera_id cam id
 * @return string route name
 * @example "logi0" for camera 0, resulting in rtsp://localhost:8554/logi0 targeted at the media server, and therefore http://192.168.1.69:8889/logi0 as the GUI link
 */
static string stream_name_from_id(int camera_id) {
    if (camera_id == -1) {
        return "wrist";
    } else {
        return "logi" + to_string(camera_id);
    }
}

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // H264 ffmpeg string
    string command = string() +
    "ffmpeg -y -f rawvideo -pix_fmt bgr24" + // ffmpeg, rawvideo format, pixel format BGR-24bit(8 per), size
    " -s " + to_string(int(set.width)) + "x" + to_string(int(set.height)) + // size string
    " -re" + // Accept frames at rate received (no set frame rate)
    " -i -" + // Pipe input (- short for pipe:0)
    " -c:v libx264 -preset veryfast -tune zerolatency" + // libx264, verfast, zerolatency tune
    // TODO bitrate
    " -f rtsp -rtsp_transport tcp " + RTSP_TRANSPORT_ENDPOINT + stream_name_from_id(camera_id) + // RTSP link
    " -loglevel quiet"; // Silence terminal output

    // Open pipe as write - we can read with the fifo
    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        // FIXME either this or the loop needs to be retrying if they find nullptr
        // TODO debug message should also be sent
        return nullptr;
    }

    return pipe;
}

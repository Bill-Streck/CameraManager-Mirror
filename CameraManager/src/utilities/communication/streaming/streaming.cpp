/**
 * @file streaming.cpp
 * @author William Streck
 * @brief ffmpeg streaming implementation.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "streaming.hpp"
#include <sys/stat.h>
#include <fcntl.h>

// TODO file-scope index of existing pipes
// TODO function accessible globally for cleaning pipes from the index (on end and shutdown)
// TODO same function on startup for cleaning purposes

static std::string make_pipe_name(int camera_id) {
    return "/tmp/camera" + std::to_string(camera_id) + ".fifo";
}

std::tuple<FILE*, int> ffmpeg_stream_camera(settings set, int camera_id) {
    // std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    // std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) +
    // " -r " + std::to_string(set.fps) + " -i - -c:v libsvtav1 -preset " + AV1_PRESET +
    // " -f matroska udp://localhost:9999";

    // Create the pipe (fifo) for the ffmpeg process to communicate back to our server
    std::string pipe_name = make_pipe_name(camera_id);
    mkfifo(pipe_name.c_str(), FIFO_MODE);

    // H264 fallback (5-6x the bitrate, but much lower latency)
    std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) +
    // TODO constants
    " -re" +
    " -i - " + // Pipe input
    "-c:v libx264 -preset veryfast -tune zerolatency" +
    " -f mpegts -omit_video_pes_length 0 " + pipe_name;

    // Open pipe as write - we can read with the fifo
    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        std::cerr << "Error opening pipe for ffmpeg" << std::endl;
        return {nullptr, -1};
    }

    // Open the pipe for reading from ffmpeg
    int inputfd = open(pipe_name.c_str(), O_RDONLY);

    return {pipe, inputfd};
}

// TODO server thread with parameters (object pointers, or we could just extern them)
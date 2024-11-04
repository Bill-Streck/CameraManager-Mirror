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
    std::string command = "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + 
    std::to_string(set.width) + "x" + std::to_string(set.height) +
    " -r " + std::to_string(set.fps) + " -i -pipe:0 -an c:v libsvtav1 -preset " + AV1_PRESET +
    " udp://localhost:1234"; // TODO actual outputs (no idea how to choose these yet :) )

    FILE* pipe = popen(command.c_str(), "w");
    std::cout<<"tried"<<std::endl;
    if (!pipe) {
        std::cerr << "Error opening pipe for ffmpeg" << std::endl;
        return nullptr;
    }
}
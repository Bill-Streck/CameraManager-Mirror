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

int local_port_from_camera_id(int camera_id) {
    return LOCAL_PORT_BASE + camera_id;
}

int stream_port_from_camera_id(int camera_id) {
    return STREAM_PORT_BASE + camera_id;
}

FILE* ffmpeg_stream_camera(settings set, int camera_id) {
    // H264 ffmpeg string
    std::string command = 
    "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s " + // ffmpeg, rawvideo format, pixel format BGR-24bit(8 per), size
    std::to_string(int(set.width)) + "x" + std::to_string(int(set.height)) + // size string
    " -re" + // TODO check what this is again???
    " -i -" + // Pipe input (- short for pipe:0)
    " -c:v libx264 -preset veryfast -tune zerolatency" + // libx264, verfast, zerolatency tune
    " -f mpegts -omit_video_pes_length 0 " + // TODO investigate is omit pes is causing the issue
    "udp://" + LOCALHOST_IP_ADDR + ":" + std::to_string(local_port_from_camera_id(camera_id)) // output IP
    + " -loglevel quiet"; // Silence terminal output

    // Open pipe as write - we can read with the fifo
    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        // FIXME either this or the loop needs to be retrying if they find nullptr
        // TODO debug message should also be sent
        return nullptr;
    }

    // [ ] make note where relevant: pipe kill is taken care of in command_handler

    return pipe;
}

int initialize_stream_socket(void) {
    // FIXME error handling (-1)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        // handle
    }

    int broadcast_enabled = 1;

    // FIXME error handling (-1)
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enabled, sizeof(broadcast_enabled)) < 0) {
        // handle
    }

    return sock;
}

int initialize_local_socket(int local_port) {
    // FIXME error handling (-1)
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        // handle
    }

    // Set to non blocking modes (without destroying old flags)
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // Addressing
    sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY); // I think this means localhost
    serveraddr.sin_port = htons(local_port);

    if (bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        // FIXME handle
        close(sock);
    }

    return sock;
}

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
#include "settings.hpp"
#include "startup.hpp"

#define AV1_PRESET "11" ///< AV1 preset for ffmpeg.
#define FIFO_MODE 0666 ///< Mode for IPC FIFO. 6 -> 4 (read) + 2 (write) + 0 (no execution); x3 -> owner, groups, others.
#define STREAM_PORT_BASE 53838 ///< Port base. Ensure nobody is using this range
#define BROADCAST_IP_ADDR "255.255.255.255" ///< Standard UNIX broadcasting IP address
#define LOCALHOST_IP_ADDR "127.0.0.1" ///< Standard localhost network target
#define BROADCAST_ENABLED 1 ///< Just represents a true broadcast parameter value

/**
 * @brief Starts the server thread.
 * 
 */
void start_server(void);

/**
 * @brief Cleans server resources.
 * 
 */
void end_server(void);

/**
 * @brief Starts the ffmpeg streaming process for a camera. Consumes the device resource.
 * 
 * @param set Settings object for the camera.
 * @param camera_id Camera device index.
 * @return FILE* pointer to the pipe for the ffmpeg process. Can pclose from here. nullptr on error.
 */
FILE* ffmpeg_stream_camera(settings set, int camera_id);

// FIXME need a get stream frame data function

/**
 * @brief Simple Server class for sending data with socket utilities
 * 
 */
class SimpleServer
{
    public:
        SimpleServer();

        /**
         * @brief Sends a packet for the selected camera_id
         * 
         * @param camera_id camera_id to send
         * @param pipe_name camera pipe name
         */
        void send_packet(int camera_id, std::string pipe_name);

    private:
        /**
         * @brief Socket identifier for sending data
         * 
         */
        int sock;

        /**
         * @brief Creates a socket for broadcasting data
         * 
         * @return int 
         */
        int initialize_socket(void);
};

#endif

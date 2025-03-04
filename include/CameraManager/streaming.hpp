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

#define MPEGTS_PACKET_MAX 1472 ///< Maximum size of an MPEG-TS packet in bytes on the UDP interface.
#define AV1_PRESET "11" ///< AV1 preset for ffmpeg.
#define FIFO_MODE 0666 ///< Mode for IPC FIFO. 6 -> 4 (read) + 2 (write) + 0 (no execution); x3 -> owner, groups, others.
#define STREAM_PORT_BASE 53838 ///< Port base. Ensure nobody is using this range on the base station.
#define LOCAL_PORT_BASE 33838 ///< Localhost port base. Ensure nobody is using this range locally.
#define BROADCAST_IP_ADDR "255.255.255.255" ///< Standard UNIX broadcasting IP address
#define LOCALHOST_IP_ADDR "127.0.0.1" ///< Standard localhost network target
#define BROADCAST_ENABLED 1 ///< Just represents a true broadcast parameter value

/**
 * @brief Makes the lcoal stream port number for a camera to be fed into ffmpeg
 * 
 * @param camera_id Camera id
 * @return int Local port number
 */
int local_port_from_camera_id(int camera_id);

/**
 * @brief Makes the remote stream port number for a camera to be broadcasted
 * 
 * @param camera_id Camera id
 * @return int Stream port number
 */
int stream_port_from_camera_id(int camera_id);

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
 * @brief Creates a socket for broadcasting data
 * 
 * @return int Socket descriptor
 */
int initialize_stream_socket(void);

/**
 * @brief Creates a socket for listening to a camera
 * 
 * @param local_port Local port to listen on
 * @return int Socket descriptor
 */
int initialize_local_socket(int local_port);

#endif

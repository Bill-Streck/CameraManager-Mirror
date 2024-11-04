/**
 * @file command_handler.hpp
 * @author William Streck
 * @brief command handler structure.
 * Command handler should have utilities for thread management and command execution over zmq.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <zmq.hpp>
#include "utilities/cameras/camera_object.hpp"

// XXX might be nice to have a debug ZMQ channel for string messages so the other one doesn't have to care about types

/******* ZMQ addresses *******/
/***** TCP is fine here *****/

#define ZMQ_LOCAL_REC "tcp://localhost:5555" ///< Local receiver address. Not used by the command handler.
#define ZMQ_LOCAL_PUB "tcp://*:6666" ///< Local publisher address.

// [ ] These might actually be fine and we just use ROS :)
#define ZMQ_REMOTE_REC "tcp://localhost:7777"
#define ZMQ_REMOTE_PUB "tcp://*:8888"

/******* Command ids *******/

#define LOCAL_START '0' ///< Local camera start command. @example 0qu10id05
#define STREAM_START '1' ///< Stream camera start command. @example 1qu10id05
#define LOCAL_STOP '2' ///< Local camera stop command. Will take down a stream with it if the camera is multi-tasking. @example 205
#define STREAM_STOP '3' ///< Stream camera stop command. @example 305

/**
 * @brief Initializes command handler data structures and loops.
 * 
 */
void init_command_handler(void);

/**
 * @brief Starts the command master thread.
 * 
 */
void begin_handler_loop(void);

/**
 * @brief Cleans all zmq sockets, context, and threaded resources.
 * 
 */
void clean_command_handler(void);

#endif

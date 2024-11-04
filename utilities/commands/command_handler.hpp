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

// TODO might be nice to have a debug ZMQ channel for string messages so the other one doesn't have to care about types

#define ZMQ_LOCAL_REC "tcp://localhost:5555" ///< Local receiver address. Not used by the command handler.
#define ZMQ_LOCAL_PUB "tcp://*:6666" ///< Local publisher address.

// TODO when chance, assess stability to see if we need tcp or can just use udp
#define ZMQ_REMOTE_REC "tcp://localhost:7777" // TODO Remote receiver address.
#define ZMQ_REMOTE_PUB "tcp://*:8888" // TODO Remote publisher address.

// commands

#define LOCAL_START '0' ///< Local camera start command.
#define STREAM_START '1' ///< Stream camera start command.

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

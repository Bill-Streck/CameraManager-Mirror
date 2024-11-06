/**
 * @file CM_include.hpp
 * @author William Streck
 * @brief Parent include for CameraManager main.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#ifndef CM_INCLUDE_HPP
#define CM_INCLUDE_HPP

#include <iostream>
#include <thread>
#include "utilities/cameras/camera_object.hpp"
#include "utilities/commands/command_handler.hpp"
#include "utilities/commands/command_board.hpp"

#define ZMQ_REMOTE_REC "tcp://localhost:7777"
#define ZMQ_REMOTE_PUB "tcp://*:8888"

#endif

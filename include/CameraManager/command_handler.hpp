/**
 * @file command_handler.hpp
 * @author William Streck
 * @brief command handler structure.
 * Command handler should have utilities for thread management.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "camera_object.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;

#define CAMERA_ID_FAIL -400 ///< Failure identifier for attaining a camera ID

/****** Sleep Macros ******/

#define HANDLER_MESSAGE_SLEEP 10 ///< Sleep time for the handler loop in milliseconds when messages are present.
#define HANDLER_NO_MESSAGE_SLEEP 100 ///< Sleep time for the handler loop in milliseconds when no messages are present.
#define CAMERA_FAILURE_RETRY 400 ///< Sleep time for the handler loop in milliseconds when a camera fails to start.

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
 * @brief Cleans all threaded or held resources.
 * 
 */
void clean_command_handler(void);

#endif

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

#include "camera_thread.hpp"
#include "rclcpp/rclcpp.hpp"

/****** Sleep Macros ******/

#define HANDLER_MESSAGE_SLEEP 10 ///< Sleep time for the handler loop in milliseconds when messages are present.
#define HANDLER_NO_MESSAGE_SLEEP 100 ///< Sleep time for the handler loop in milliseconds when no messages are present.

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

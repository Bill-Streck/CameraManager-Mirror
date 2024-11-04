/**
 * @file command_board.hpp
 * @author William Streck
 * @brief Command board utility functions.
 * @version 0.1
 * @date 2024-11-04
 * 
 */

#ifndef COMMAND_BOARD_HPP
#define COMMAND_BOARD_HPP

#include <iostream>

/**
 * @brief Post a command to the command board queue.
 * 
 * @param command command to post for the command handler to process
 */
void post_command(std::string command);

/**
 * @brief Get the next command from the command board queue.
 * 
 * @return std::string next command to process
 */
std::string get_command();

#endif

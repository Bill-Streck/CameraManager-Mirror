/**
 * @file command_generation.hpp
 * @author William Streck
 * @brief Command generation and interpretation utilities for the Camera Manager.
 * @version 0.1
 * @date 2024-11-10
 * 
 */

#ifndef COMMAND_GENERATION_HPP
#define COMMAND_GENERATION_HPP

#include <string>
#include <map>

/**
 * @brief Handles the raw ROS2 command.
 * 
 * @param command ROS2 message to handle.
 */
void handle_command(uint32_t command);

/**
 * @brief Parse a command string into a map of key-value pairs.
 * To be used by the command handler and its threads.
 * Currently dependent on string being correct (internally handled - should not be used to parse messages).
 * 
 * @param command The command string to parse.
 * @return std::map<std::string, std::string> The parsed key-value pairs.
 * @example 0qu10id05 -> {"qu": "10", "id": "05"} with command type 0 -> local start camera 5 with quality 10
 */
std::map<std::string, std::string> parse_cmd(std::string command);

#endif

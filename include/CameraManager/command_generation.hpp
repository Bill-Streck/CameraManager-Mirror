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

using namespace std;

/******* Command ids *******/

#define LOCAL_START "0" ///< Local camera start command. @example 0qu10id05
#define STREAM_START "1" ///< Stream camera start command. @example 1qu10id05
#define LOCAL_STOP "2" ///< Local camera stop command. Will take down a stream with it if the camera is multi-tasking. @example 205
#define STREAM_STOP "3" ///< Stream camera stop command. @example 305
#define FORCE_RESTART "4" ///< Force restart command. Retains state. @example 405
#define ATTRIBUTE_MODIFY "5" ///< Attribute modification command. @example 5qu10id05

#define INDEX_MODE "m" ///< Mode index in parsed command.
#define INDEX_QUALITY "q" ///< Quality index in parsed command.
#define INDEX_ID "i" ///< Numeric camera index in parsed command.
#define INDEX_ATTRIBUTE "a" ///< Attribute index in parsed command.
#define INDEX_AT_VALUE "v" ///< Value index in parsed command.
#define WRIST_ID "wr" ///< Special id for wrist camera.
#define AUX_INDEX_BASE "base" ///< Special command handler ID for command map bases.
#define COMMAND_MAP_END "end" ///< Special base end camera thread command.

#define FAIL_RET map<string, string>()

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
 * @return map<string, string> The parsed key-value pairs.
 * @example 0qu10id05 -> {"qu": "10", "id": "05"} with command type 0 -> local start camera 5 with quality 10
 */
map<string, string> parse_cmd(string command);

#endif

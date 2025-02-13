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
#include <vector>

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
#define WRIST_ID_BIN 0b11111 ///< Binary value representing wrist id.
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
 * @brief Used for prestarting cameras on startup
 * 
 */
void prestart_cameras(std::vector<int64_t> prestarts, int64_t qual);

#endif

/**
 * @file command_generation.cpp
 * @author William Streck
 * @brief Procedures for generating and parsing commands for the Camera Manager based on ROS2 messages.
 * @version 0.1
 * @date 2024-11-10
 * 
 */

#include "command_generation.hpp"
#include "command_board.hpp"
#include "command_handler.hpp"
#include "settings.hpp"

/**
 * @brief Generates a command string from the raw ROS2 command.
 * 
 * @param command 
 * @return std::string 
 */
static map<string, string> generate_command(uint32_t command) {
    map<string, string> parsed;

    auto command_type = command >> 29; // 31:29
    auto short_id = (command >> 24) & 0b11111; // 28:24 - for short commands
    auto long_quality = (command >> 24) & 0b11111; // 28:24 - for long commands
    auto long_id = (command >> 19) & 0b11111; // 23:19 - for long commands
    auto attribute = (command >> 20) & 0b1111; // 23:20
    // Now get the value from bits 16:0 (yes, we did skip some) for the modification
    auto value = command & 0xFFFF; // 16:0
    bool is_long_command = false; // if the command uses "long" parsing
    bool uses_quality = false; // if the command uses quality
    switch (command_type) {
        case 0:
            parsed[INDEX_MODE] = LOCAL_START;
            is_long_command = true;
            uses_quality = true;
            break;
        case 1:
            parsed[INDEX_MODE] = STREAM_START;
            is_long_command = true;
            uses_quality = true;
            break;
        case 2:
            parsed[INDEX_MODE] = LOCAL_STOP;
            break;
        case 3:
            parsed[INDEX_MODE] = STREAM_STOP;
            break;
        case 4:
            parsed[INDEX_MODE] = FORCE_RESTART;
            break;
        case 5:
            // id done, now get the attribute from bits 23:20 (4 bits)
            if (attribute > 6) {
                // Invalid attribute (only 7 available) - do not handle
                return FAIL_RET;
            }

            /* Note attribute can be quality, brightness, contrast, saturation, sharpness, gain, or auto white balance.
             * Be mindful this means the command will have to be handled outside.
             */
            parsed[INDEX_MODE] = ATTRIBUTE_MODIFY;
            parsed[INDEX_ATTRIBUTE] = to_string(attribute);
            parsed[INDEX_AT_VALUE] = to_string(value);
            break;
        default:
            // Invalid command - do not handle
            return FAIL_RET;
    }

    if (is_long_command) {
        // Long ID
        if (long_id == 0b11111) {
            parsed[INDEX_ID] = WRIST_ID;
        } else {
            parsed[INDEX_ID] = long_id;
        }

        // Quality
        if (uses_quality) {
            if (long_quality > CLARITY_PRESETS) {
                // Invalid quality - do not handle
                return FAIL_RET;
            }

            parsed[INDEX_QUALITY] = to_string(long_quality);
        }
    } else {
        // Just get the short ID
        if (short_id == 0b11111) {
            parsed[INDEX_ID] = WRIST_ID;
        } else {
            parsed[INDEX_ID] = to_string(short_id);
        }
    }

    return parsed;
}

void handle_command(uint32_t command) {
    // Parse the command and post it
    post_command(generate_command(command));
}

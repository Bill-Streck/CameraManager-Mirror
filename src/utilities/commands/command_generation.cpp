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
static map<string, string> generate_command(robot_interfaces::msg::CameraManagerCommand::SharedPtr command) {
    map<string, string> parsed;

    auto command_type = command->command_type;
    auto camera_id = command->camera_id;
    auto quality = command->quality;
    auto attribute = command->attribute;
    // Now get the value from bits 16:0 (yes, we did skip some) for the modification
    auto attr_value = command->attr_value;
    bool uses_quality = false; // if the command uses quality
    switch (command_type) {
        case 0:
            parsed[INDEX_MODE] = LOCAL_START;
            break;
        case 1:
            parsed[INDEX_MODE] = STREAM_START;
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
            if (attribute > 6) {
                // Invalid attribute (only 7 available) - do not handle
                return FAIL_RET;
            }

            /* Note attribute can be quality, brightness, contrast, saturation, sharpness, gain, or auto white balance.
             * Be mindful this means the command will have to be handled outside.
             */
            parsed[INDEX_MODE] = ATTRIBUTE_MODIFY;
            parsed[INDEX_ATTRIBUTE] = to_string(attribute);
            parsed[INDEX_AT_VALUE] = to_string(attr_value);
            break;
        default:
            // Invalid command - do not handle
            return FAIL_RET;
    }

    if (uses_quality) {
        if (quality > CLARITY_PRESETS) {
            // Invalid quality - do not handle
            return FAIL_RET;
        }

        parsed[INDEX_QUALITY] = to_string(quality);
    }

    if (camera_id == WRIST_ID_NUM) {
        parsed[INDEX_ID] = WRIST_ID;
    } else {
        parsed[INDEX_ID] = to_string(camera_id);
    }

    return parsed;
}

void prestart_cameras(vector<int64_t> prestarts, vector<int64_t> qualities) {
    for (size_t i=0; i<prestarts.size(); i++) {
        auto cam_id = prestarts.at(i); // guaranteed
        int64_t qual;

        try {
            qual = qualities.at(i);
        } catch (const exception& e) {
            qual = 2; // default to 320x180 at 10fps if they don't give a quality
        }

        map<string, string> parsed;

        // We know we only do local here
        parsed[INDEX_MODE] = LOCAL_START;
        if (cam_id == WRIST_ID_NUM) {
            parsed[INDEX_ID] = WRIST_ID;
        } else {
            parsed[INDEX_ID] = to_string(cam_id);
        }

        parsed[INDEX_QUALITY] = to_string(qual);

        post_command(parsed);
    }
}

void handle_command(robot_interfaces::msg::CameraManagerCommand::SharedPtr command) {
    // Parse the command and post it
    post_command(generate_command(command));
}

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
static std::string generate_command(uint32_t command) {
    std::string command_str = "";

    auto command_type = command >> 29; // 31:29
    auto short_id = (command >> 24) & 0b11111; // 28:24 - for short commands
    auto long_quality = (command >> 24) & 0b11111; // 28:24 - for long commands
    auto long_quality_str = std::to_string(long_quality);
    auto long_id = (command >> 19) & 0b11111; // 23:19 - for long commands
    auto attribute = (command >> 20) & 0b1111; // 23:20
    auto attribute_str = std::to_string(attribute);
    // Now get the value from bits 16:0 (yes, we did skip some) for the modification
    auto value = command & 0xFFFF; // 16:0
    // CRUCIAL: safeties must be parsed elsewhere
    auto value_str = std::to_string(value);
    switch (command_type) {
        case 0:
            command_str += LOCAL_START;
            if (long_quality > CLARITY_PRESETS) {
                // Invalid quality - do not handle
                return "";
            }
            if (long_quality_str.size() == 1) {
                long_quality_str = "0" + long_quality_str;
            }
            command_str += "qu" + long_quality_str;
            if (long_id == 0b11111) {
                command_str += "idwr";
            } else {
                command_str += "id";
                auto long_id_str = std::to_string(long_id);
                if (long_id_str.size() == 1) {
                    long_id_str = "0" + long_id_str;
                }
                command_str += long_id_str;
            }
            break;
        case 1:
            command_str += STREAM_START;
            if (long_quality > CLARITY_PRESETS) {
                // Invalid quality - do not handle
                return "";
            }
            if (long_quality_str.size() == 1) {
                long_quality_str = "0" + long_quality_str;
            }
            command_str += "qu" + long_quality_str;
            if (long_id == 0b11111) {
                command_str += "idwr";
            } else {
                command_str += "id";
                auto long_id_str = std::to_string(long_id);
                if (long_id_str.size() == 1) {
                    long_id_str = "0" + long_id_str;
                }
                command_str += long_id_str;
            }
            break;
        case 2:
            command_str += LOCAL_STOP;
            if (short_id == 0b11111) {
                command_str += "wr";
            } else {
                // No "id" key for this command
                auto short_id_str = std::to_string(short_id);
                if (short_id_str.size() == 1) {
                    short_id_str = "0" + short_id_str;
                }
                command_str += short_id_str;
            }
            break;
        case 3:
            command_str += STREAM_STOP;
            if (short_id == 0b11111) {
                command_str += "wr";
            } else {
                // No "id" key for this command
                auto short_id_str = std::to_string(short_id);
                if (short_id_str.size() == 1) {
                    short_id_str = "0" + short_id_str;
                }
                command_str += short_id_str;
            }
            break;
        case 4:
            command_str += FORCE_RESTART;
            if (short_id == 0b11111) {
                command_str += "wr";
            } else {
                // No "id" key for this command
                auto short_id_str = std::to_string(short_id);
                if (short_id_str.size() == 1) {
                    short_id_str = "0" + short_id_str;
                }
                command_str += short_id_str;
            }
            break;
        case 5:
            /* Note attribute can be quality, brightness, contrast, saturation, sharpness, gain, or auto white balance.
             * Be mindful this means the command will have to be handled outside.
             * This 
             */
            command_str += ATTRIBUTE_MODIFY;
            if (short_id == 0b11111) {
                command_str += "idwr";
            } else {
                command_str += "id";
                auto short_id_str = std::to_string(short_id);
                if (short_id_str.size() == 1) {
                    short_id_str = "0" + short_id_str;
                }
                command_str += short_id_str;
            }

            // id done, now get the attribute from bits 23:20 (4 bits)
            if (attribute > 6) {
                // Invalid attribute (only 7 available) - do not handle
                return "";
            }
            if (attribute_str.size() == 1) {
                attribute_str = "0" + attribute_str;
            }
            command_str += "at" + attribute_str;

            // pad value to 5 digits
            while (value_str.size() < 5) {
                value_str = "0" + value_str;
            }
            command_str += "va" + value_str;
            break;
        default:
            // Invalid command - do not handle
            return "";
    }

    std::cout << "Generated command: " << command_str << std::endl;
    return command_str;
}

void handle_command(uint32_t command) {
    // Parse the command
    auto command_str = generate_command(command);

    post_command(command_str);
}

std::map<std::string, std::string> parse_cmd(std::string command) {
    std::map<std::string, std::string> parsed;
    
    // Erase the first character as it is the command type
    command.erase(0, 1);

    // Find key-value pairs
    size_t pos = 0;
    while (pos < command.size()) {
        std::string key = command.substr(pos, 2);
        pos += 2;
        std::string value = command.substr(pos, 2); // FORCES TWO DIGIT NUMBERS
        parsed[key] = value;
        pos += 2; // to next pair
    }

    return parsed;
}
/**
 * @file command_board.cpp
 * @author William Streck
 * @brief Command board data structure and functions.
 * @version 0.1
 * @date 2024-11-04
 * 
 */

#include "command_board.hpp"
#include <queue>

std::queue<std::string> command_board;

void post_command(std::string command) {
    command_board.push(command);
}

std::string get_command() {
    if (command_board.empty()) {
        return "";
    }
    std::string command = command_board.front();
    command_board.pop(); // No idea who wrote the queue library but this isn't their brightest work I'd hope.
    return command;
}

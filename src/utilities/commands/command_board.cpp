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
#include <mutex>

queue<map<string, int>> command_board;
static mutex command_board_mutex;

void post_command(map<string, int> command) {
    lock_guard<mutex> lock(command_board_mutex); // Automatically unlocks when out of scope
    command_board.push(command);
}

map<string, int> get_command() {
    lock_guard<mutex> lock(command_board_mutex); // Automatically unlocks when out of scope
    if (command_board.empty()) {
        return map<string, int>();
    }
    map<string, int> command = command_board.front();
    command_board.pop(); // No idea who wrote the queue library but this isn't their brightest work I'd hope.
    return command;
}

/**
 * @file command_handler.cpp
 * @author William Streck
 * @brief Command handler implementation, including static functions for use as threads.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "command_handler.hpp"
#include "command_board.hpp"
#include "realsense_interface.hpp"
#include <iostream>
#include <thread>
#include <mutex>

// TODO split these functions out into different files

static map<int, thread> threads; ///< Map of threads for command execution.

int map_counter = 0; ///< Counter for the thread map. Not right to count up, but if you find a way to overflow let me know because that's impressive.

void init_command_handler(void) {
    // Start thread handler loop
    begin_handler_loop();
}

static void handler_loop() {
    while (running) {
        // Quickly clean any one terminated thread
        if (threads_end.size() > 0) {
            int clear = threads_end.front();
            threads[clear].join();
            threads.erase(clear);
            threads_end.remove(clear);
        }

        map<string, int> parsed = get_command();
        if (parsed.size() == 0) {
            // Command doesn't exist
            this_thread::sleep_for(chrono::milliseconds(HANDLER_NO_MESSAGE_SLEEP));
            continue;
        }

        // Parse the command for usage, also every command requires id
        int id = parsed[INDEX_ID];

        if (parsed[INDEX_MODE] == LOCAL_START) {
            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                if (parsed[INDEX_ID] == WRIST_ID) {
                    // FIXME not implemented yet - start intel thread
                } else {
                    // Start the normal cam thread
                    threads.insert(pair<int, thread>(map_counter, thread(logi_cam_thread, parsed, map_counter)));
                }
                map_counter++;
                // TODO uhhhhhhhhh why is there a camera object here???
                auto camera = Camera();
                cameras.insert(pair<int, Camera>(id, camera));
            }

            // Tell the camera to start locally if it isn't already
            if (local_cams.find(id) == local_cams.end()) {
                // TODO we need to address resolution in this case
                local_cams.insert(pair<int, string>(id, " "));
            }
        } else if (parsed[INDEX_MODE] == STREAM_START) {
            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                if (parsed[INDEX_ID] == WRIST_ID) {
                    // Start the realsense thread
                    threads.insert(pair<int, thread>(map_counter, thread(realsense_cam_thread, parsed, map_counter)));
                } else {
                    // Start the normal cam thread
                    threads.insert(pair<int, thread>(map_counter, thread(logi_cam_thread, parsed, map_counter)));
                }
                map_counter++;
                auto camera = Camera();
                cameras.insert(pair<int, Camera>(id, camera));
            }
            
            // Tell the camera to start streaming if it isn't already
            if (streaming_cams.find(id) == streaming_cams.end()) {
                // TODO we need to address resolution in this case
                streaming_cams.insert(pair<int, string>(id, " "));
            }
        } else if (parsed[INDEX_MODE] == LOCAL_STOP) {
            // Remove the camera from local cams
            if (local_cams.find(id) != local_cams.end()) {
                local_cams.erase(id);
            }

            // If the camera is not streaming, stop it entirely to save resources
            if (streaming_cams.find(id) == streaming_cams.end()) {
                parsed[AUX_INDEX_BASE] = COMMAND_MAP_END;
                cam_command_map[id] = parsed;
            }
        } else if (parsed[INDEX_MODE] == STREAM_STOP) {
            // Remove camera from streaming cams
            if (streaming_cams.find(id) != streaming_cams.end()) {
                streaming_cams.erase(id);
            }

            // If the camera is not being used locally, stop it entirely to save resources
            if (local_cams.find(id) == local_cams.end()) {
                parsed[AUX_INDEX_BASE] = COMMAND_MAP_END;
                cam_command_map[id] = parsed;
            }
        } else if (parsed[INDEX_MODE] == FORCE_RESTART) {
            // Stop local running
            if (local_cams.find(id) != local_cams.end()) {
                local_cams.erase(id);
            }

            // Stop streaming
            if (streaming_cams.find(id) != streaming_cams.end()) {
                streaming_cams.erase(id);
            }

            // Stop thread
            parsed[AUX_INDEX_BASE] = COMMAND_MAP_END;
            cam_command_map[id] = parsed;
        } else if (parsed[INDEX_MODE] == ATTRIBUTE_MODIFY) {
            if (local_cams.find(id) != local_cams.end() || streaming_cams.find(id) != streaming_cams.end()) {
                if (cam_command_map.find(id) == cam_command_map.end()) {
                    cam_command_map[id] = parsed;
                } else {
                    // Only overwrite if the command is not end
                    if (cam_command_map[id].find(AUX_INDEX_BASE) != cam_command_map[id].end()
                    && cam_command_map[id][AUX_INDEX_BASE] != COMMAND_MAP_END) {
                        parsed[AUX_INDEX_BASE] = COMMAND_MAP_END;
                        cam_command_map[id] = parsed;
                    }
                }
            }
        }

        // Miniscule efficiency improvement - we sleep less here in case there are more commands
        this_thread::sleep_for(chrono::milliseconds(HANDLER_MESSAGE_SLEEP));
    }
}

void begin_handler_loop() {
    // Start the handler loop in a new thread
    threads.insert(pair<int, thread>(-1, thread(handler_loop)));
}

void clean_command_handler(void) {
    // Signal end for all threads
    running = false;

    // Join all threads (that should have been terminated already)
    for (auto& thread : threads) {
        // ffmpeg processes die with this application
        thread.second.join();
    }
}



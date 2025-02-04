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
#include "streaming.hpp"
#include "command_generation.hpp"
#include "command_board.hpp"
#include "CameraManagerNode.hpp"
#include <iostream>
#include <thread>
#include <mutex>

// TODO split these functions out into different files

// Compatibility with certain development configurations (e.g. Wanderer2)
#ifndef SIGKILL
    #include <signal.h>
#endif

extern shared_ptr<CameraManager> camera_manager_node;

static map<int, Camera> cameras; ///< Map of camera objects. Used for seeing if a camera is on.
static map<int, string> local_cams; ///< Map of cameras that are being used locally.
static map<int, string> streaming_cams; ///< Map of cameras that are streaming. Used for ffmpeg process management.

static map<int, map<string, string>> cam_command_map; ///< Map of end flags for each thread, regardless of type.

static map<int, thread> threads; ///< Map of threads for command execution.
static list<int> threads_end; ///< List of threads that have ended.

static mutex publisher_mutex; ///< Mutex for local publisher socket.

static bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.

int map_counter = 0; ///< Counter for the thread map. Not right to count up, but if you find a way to overflow let me know because that's impressive.

static int find_cam_id(map<string, string> parsed) {
    int camera_id = CAMERA_ID_FAIL;
    if (parsed[INDEX_ID] == WRIST_ID) {
        camera_id = -1;
    } else {
        try {
            camera_id = stoi(parsed[INDEX_ID]);
        } catch (const exception& e) {/* Do nothing */}
    }

    return camera_id;
}

static clarity preset_from_quality(int quality) {
    if (quality == 0) {
        return lowest;
    } else if (quality == 1) {
        return low;
    } else if (quality == 2) {
        return lowish;
    } else if (quality == 3) {
        return okay;
    } else if (quality == 4) {
        return okayish;
    } else if (quality == 5) {
        return medium;
    } else if (quality == 6) {
        return mediumish;
    } else if (quality == 7) {
        return high;
    } else if (quality == 8) {
        return higher;
    } else if (quality == 9) {
        return highest;
    }

    // fallback
    return okay;
}

void init_command_handler(void) {
    // Start thread handler loop
    begin_handler_loop();
}

// TODO rename and comment
static void local_camera_start(map<string, string> parsed, int tmap_index) {
    // Pull the important values from the command
    auto quality = stoi(parsed[INDEX_QUALITY]);
    int camera_id = find_cam_id(parsed);

    if (camera_id == CAMERA_ID_FAIL || camera_id > MAX_CAMERA_ID) {
        // Camera is already on
        // TODO we should send a string message back to the client
        threads_end.push_back(tmap_index); // Clean thread
        return;
    }

    auto camera = Camera();
    auto set = settings();
    set.device_index = camera_id;
    auto preset = preset_from_quality(quality);
    set.use_preset(preset);

    camera.configure(set);

    try {
        camera.start();
    } catch(const exception& e) {
        // Please note this will be caught and handled inside the while loop - I am avoiding duplicate code
    }

    // Get basic camera variables ready
    cameras[camera_id] = camera;
    FILE* pipe = nullptr;
    auto cam_streaming = false;
    auto cam_local = false;

    // Specific members for streaming
    // FIXME still need a small amount of logic for safely closing all the sockets
    int stream_port = stream_port_from_camera_id(camera_id);
    int local_port = local_port_from_camera_id(camera_id);
    sockaddr_in broadcast_address;
    broadcast_address.sin_family = AF_INET;
    broadcast_address.sin_addr.s_addr = inet_addr(BROADCAST_IP_ADDR);
    broadcast_address.sin_port = htons(stream_port);
    int stream_sockfd = initialize_stream_socket();
    int local_sockfd = initialize_local_socket(local_port);

    // Capture loop
    while (running) {
        // Check if we should be streaming or should close a stream
        if (!cam_streaming && streaming_cams.find(camera_id) != streaming_cams.end()) {
            pipe = ffmpeg_stream_camera(set, camera_id);
            if (pipe == nullptr) {
                // TODO send a verification message (or error)
                
                // Erase from streaming cams for now
                if (streaming_cams.find(camera_id) != streaming_cams.end()) {
                    streaming_cams.erase(camera_id);
                }
            }
            cam_streaming = true;
        } else if (cam_streaming && streaming_cams.find(camera_id) == streaming_cams.end()) {
            // Stop the ffmpeg process
            if (pipe != nullptr) {
                // kill
                kill(fileno(pipe), SIGKILL);
                pclose(pipe);
                pipe = nullptr;
            }
            cam_streaming = false; // pipe was closed somehow I guess
        }

        // Check if we should be running local comms
        if (!cam_local && local_cams.find(camera_id) != local_cams.end()) {
            cam_local = true;
        } else if (cam_local && local_cams.find(camera_id) == local_cams.end()) {
            cam_local = false;
        }
        
        // Get the frame
        auto frame = camera.get_current_frame();

        // Get the timestamp (after frame should be most accurate)
        auto now = chrono::system_clock::now();
        auto now_seconds = chrono::time_point_cast<chrono::seconds>(now);
        auto now_nanoseconds = chrono::duration_cast<chrono::nanoseconds>(now - now_seconds);
        auto ts_seconds = static_cast<uint32_t>(now_seconds.time_since_epoch().count());
        auto ts_nanoseconds = static_cast<uint32_t>(now_nanoseconds.count());

        if (frame.empty()) {
            // Camera has no frame and must be restarted
            try {
                camera.start();
            } catch(const exception& e) {
                // TODO exchance for debug channel message
            }
            if (cam_command_map.find(camera_id) != cam_command_map.end()
            && cam_command_map[camera_id][AUX_INDEX_BASE] == COMMAND_MAP_END) {
                // TODO send a verification message

                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            }
            
            // Avoid the slamming of CPU usage for no reason
            this_thread::sleep_for(chrono::milliseconds(CAMERA_FAILURE_RETRY));
            continue; // don't need to publish an empty frame
        }

        // Publish the frame locally if applicable immediately to avoid compression delays
        if (cam_local) {
            lock_guard<mutex> lock(publisher_mutex); // Automatically unlocks when out of scope (each loop)
            // Put the image into a ROS2 message
            auto msg = sensor_msgs::msg::Image();
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = "bgr8"; // pixel encoding
            msg.step = frame.step;
            // Don't worry about endian leave it default
            msg.data = vector<uint8_t>(frame.data, frame.data + frame.total() * frame.elemSize());

            // Information in header
            msg.header.stamp.sec = ts_seconds;
            msg.header.stamp.nanosec = ts_nanoseconds;
            msg.header.frame_id = to_string(camera_id);

            // metadata
            auto meta_msg = robot_interfaces::msg::ImageMetadata();
            meta_msg.im_height = frame.rows;
            meta_msg.im_width = frame.cols;
            if (camera_id >= 0 && camera_id <= MAX_CAMERA_ID) {
                meta_msg.sensor_height = 4.0;
                meta_msg.foc_len_mm = FOCAL_LENGTH_MM;
            } else {
                meta_msg.sensor_height = 0;
                meta_msg.foc_len_mm = 0;
            }

            // Publish the messages
            camera_manager_node->publish_image(msg);
            camera_manager_node->publish_img_meta(meta_msg);
        }

        // Stream if applicable
        if (cam_streaming && (pipe != nullptr)) {
            // Write the current frame for production
            fwrite(frame.data, 1, frame.total() * frame.elemSize(), pipe);

            ssize_t recv_len = 0L;

            do {
                // Try and find data. This may not be the most recent frame - that's fine.
                // TODO confirm the max buffer size is somewhere around 1500
                char buffer[90'000]; // Storage buffer
                sockaddr_in client_addr; // Receive address marker
                socklen_t client_addr_len = sizeof(client_addr);

                // Receive data (note ssize_t is signed)
                recv_len = recvfrom(local_sockfd, buffer, sizeof(buffer), 0, 
                (sockaddr*)&client_addr, &client_addr_len);
                if (recv_len > 0) {
                    // Forward the data on the broadcast socket
                    sendto(stream_sockfd, buffer, recv_len, 0, 
                    (sockaddr*)&broadcast_address, sizeof(broadcast_address));
                }
            } while (recv_len >= 1472);
        }

        // Handle a command if one is present
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            auto cmd = cam_command_map[camera_id];
            if (cmd[AUX_INDEX_BASE] == COMMAND_MAP_END) {
                // TODO send a verification message

                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            } else if (cmd[INDEX_MODE] == ATTRIBUTE_MODIFY) {
                // Handle attribute modification
                // TODO send verification message
                auto attribute = stoi(cmd[INDEX_ATTRIBUTE]);
                auto value = stoi(cmd[INDEX_AT_VALUE]);
                // Warn them if they try gain
                if (attribute == ATTR_GAIN) {
                    // TODO warn them even though you'll still try
                }
                if (!camera.change_attribute(attribute, value)) {
                    // TODO send failure message
                }

                // Remove the command from the map
                cam_command_map.erase(camera_id);
            }
        }
    }
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

        map<string, string> parsed = get_command();
        if (parsed.size() == 0) {
            // Command doesn't exist
            this_thread::sleep_for(chrono::milliseconds(HANDLER_NO_MESSAGE_SLEEP));
            continue;
        }

        // Parse the command for usage, also every command requires id
        int id = find_cam_id(parsed);

        if (parsed[INDEX_MODE] == LOCAL_START) {
            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                threads.insert(pair<int, thread>(map_counter, thread(local_camera_start, parsed, map_counter)));
                map_counter++;
                auto camera = Camera();
                cameras.insert(pair<int, Camera>(id, camera));
            }

            // Tell the camera to start locally if it isn't already
            if (local_cams.find(id) == local_cams.end()) {
                local_cams.insert(pair<int, string>(id, " "));
            }
        } else if (parsed[INDEX_MODE] == STREAM_START) {
            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                threads.insert(pair<int, thread>(map_counter, thread(local_camera_start, parsed, map_counter)));
                map_counter++;
                auto camera = Camera();
                cameras.insert(pair<int, Camera>(id, camera));
            }
            
            // Tell the camera to start streaming if it isn't already
            if (streaming_cams.find(id) == streaming_cams.end()) {
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

    // FIXME handler cleanup (end cameras, end threads, ...)
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



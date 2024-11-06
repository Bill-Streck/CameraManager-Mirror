/**
 * @file command_handler.cpp
 * @author William Streck
 * @brief Command handler implementation, including static functions for use as threads.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "command_handler.hpp"
#include "utilities/commands/command_board.hpp"
#include "utilities/communication/streaming/streaming.hpp"
#include <iostream>
#include <thread>
#include <mutex>

static zmq::context_t context; ///< ZMQ context for communication.
static zmq::socket_t local_publisher; ///< ZMQ local publisher socket.
static zmq::socket_t subscriber; ///< ZMQ remote subscriber socket.

static std::map<int, Camera> cameras; ///< Vector of camera objects. Used for seeing if a camera is on.
static std::map<int, std::string> local_cams; ///< Map of cameras that are being used locally. Used to manage ZMQ workflow.
static std::map<int, std::string> streaming_cams; ///< Map of cameras that are streaming. Used for ffmpeg process management.

static std::map<int, std::string> cam_command_map; ///< Vector of end flags for each thread, regardless of type.

static std::map<int, std::thread> threads; ///< Vector of threads for command execution.
static std::list<int> threads_end; ///< Vector of threads that have ended.

static std::mutex local_publisher_mutex; ///< Mutex for local publisher socket.
static std::mutex remote_publisher_mutex; ///< Mutex for remote publisher socket.

static bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.

int map_counter = 0; ///< Counter for the thread map. Respectfully, if you manage to make 2^31 threads, I will be impressed.

static clarity preset_from_quality(int quality) {
    if (quality == 0) {
        return lowest;
    } else if (quality == 1) {
        return low;
    } else if (quality == 2) {
        return okay;
    } else if (quality == 3) {
        return medium;
    } else if (quality == 4) {
        return high;
    } else if (quality == 5) {
        return highest;
    }

    // fallback
    return okay;
}

static std::map<std::string, std::string> parse_cmd(std::string command) {
    // Example: 0qu10id05
    // local start command, quality 10, camera id 5
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

void init_command_handler(void) {
    // ZMQ
    context = zmq::context_t(1);

    local_publisher = zmq::socket_t(context, ZMQ_PUB);
    local_publisher.bind(ZMQ_LOCAL_PUB);
    
    subscriber = zmq::socket_t(context, ZMQ_SUB);
    subscriber.connect(ZMQ_REMOTE_REC);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    // Start thread handler loop
    begin_handler_loop();
}

static void local_camera_start(std::string command, int tmap_index) {
    // Parse the command
    auto parsed = parse_cmd(command);
    auto quality = std::stoi(parsed["qu"]);
    int camera_id = -400;
    if (parsed["id"] == "wr") {
        camera_id = -1;
    } else {
        camera_id = std::stoi(parsed["id"]);
    }

    if (cameras.find(camera_id) != cameras.end() || camera_id == -400) {
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
    std::cout << "Starting camera " << camera_id << std::endl;
    try {
        camera.start();
    } catch(const std::exception& e) {
        // Please note this will be caught and handled inside the while loop - I am avoiding duplicate code
        std::cerr << e.what() << '\n';
    }
    cameras.insert(std::pair<int, Camera>(camera_id, camera));
    FILE* pipe = nullptr;
    auto cam_streaming = false;
    auto cam_local = false;

    // Capture loop
    while (running) {
        // Check if we should be streaming or should close a stream
        if (!cam_streaming && streaming_cams.find(camera_id) != streaming_cams.end()) {
            pipe = ffmpeg_stream_camera(set, camera_id);
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

        // Check if we should be running local ZMQ comms
        if (!cam_local && local_cams.find(camera_id) != local_cams.end()) {
            cam_local = true;
        } else if (cam_local && local_cams.find(camera_id) == local_cams.end()) {
            cam_local = false;
        }
            
        auto frame = camera.get_current_frame();
        if (frame.empty()) {
            std::cout << "Camera " << camera_id << " has no frame." << std::endl;
            try
            {
                camera.start();
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            if (cam_command_map.find(camera_id) != cam_command_map.end() && cam_command_map[camera_id] == "end") {
                // TODO send a verification message
                std::cout << "Camera " << camera_id << " has been terminated." << std::endl;
                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            }
            
            // Avoid the slamming of CPU usage for no reason
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            continue; // don't need to publish an empty frame
        }

        // Publish the frame locally if applicable
        if (cam_local) {
            std::lock_guard<std::mutex> lock(local_publisher_mutex); // Automatically unlocks when out of scope (each loop)
            uchar cam_number = uchar(camera_id);
            zmq::message_t header(&cam_number, 1);
            zmq::message_t height(&frame.rows, sizeof(frame.rows));
            zmq::message_t width(&frame.cols, sizeof(frame.cols));
            zmq::message_t message(frame.data, frame.total() * frame.elemSize());
            local_publisher.send(header, ZMQ_SNDMORE);
            local_publisher.send(height, ZMQ_SNDMORE);
            local_publisher.send(width, ZMQ_SNDMORE);
            local_publisher.send(message, frame.total() * frame.elemSize());
        }

        // Stream if applicable
        if (cam_streaming) {
            if (pipe == nullptr) {
                std::cout << "Pipe is null" << std::endl;
                cam_streaming = false;
                // TODO handle in map? and let user know
                continue;
            } else {
                fwrite(frame.data, 1, frame.total() * frame.elemSize(), pipe);
            }
        }

        // Handle a command if one is present
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            auto cmd = cam_command_map[camera_id];
            if (cmd == "end") {
                // TODO send a verification message
                std::cout << "Camera " << camera_id << " has been terminated." << std::endl;
                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            }
            // TODO attribute handlers
        }
        // [ ] still might want to sleep here, but speed matters much more here than anywhere else
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
            std::cout << "Thread " << clear << " has been cleaned." << std::endl;
        }

        zmq::message_t message;
        subscriber.recv(&message, ZMQ_NOBLOCK);
        std::string command = std::string(static_cast<char*>(message.data()), message.size());
        if (command.size() == 0) {
            // Command doesn't exist
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::cout << "Received command: " << command << std::endl;
        if (command.at(0) == LOCAL_START) {
            auto parsed = parse_cmd(command);
            int id = -400;
            if (parsed["id"] == "wr") {
                id = -1;
            } else {
                id = std::stoi(parsed["id"]);
            }

            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                threads.insert(std::pair<int, std::thread>(map_counter, std::thread(local_camera_start, command, map_counter)));
                std::cout << "Thread " << map_counter << " has been created." << std::endl;
                map_counter++;
            }

            // Tell the camera to start locally if it isn't already
            if (local_cams.find(id) == local_cams.end()) {
                local_cams.insert(std::pair<int, std::string>(id, " "));
            }
        } else if (command.at(0) == STREAM_START) {
            // Parse the command for only the camera ID
            auto parsed = parse_cmd(command);
            int id = -400;
            if (parsed["id"] == "wr") {
                id = -1;
            } else {
                id = std::stoi(parsed["id"]);
            }

            // If the camera wasn't already running, start it
            if (cameras.find(id) == cameras.end()) {
                threads.insert(std::pair<int, std::thread>(map_counter, std::thread(local_camera_start, command, map_counter)));
                std::cout << "Thread " << map_counter << " has been created." << std::endl;
                map_counter++;
            }
            
            // Tell the camera to start streaming if it isn't already
            if (streaming_cams.find(id) == streaming_cams.end()) {
                streaming_cams.insert(std::pair<int, std::string>(id, " "));
            }
        } else if (command.at(0) == LOCAL_STOP) {
            auto parsed = parse_cmd(command);
            int id = -400;
            if (parsed["id"] == "wr") {
                id = -1;
            } else {
                id = std::stoi(parsed["id"]);
            }

            // Remove the camera from local cams
            if (local_cams.find(id) != local_cams.end()) {
                local_cams.erase(id);
            }

            // If the camera is not streaming, stop it entirely to save resources
            if (streaming_cams.find(id) == streaming_cams.end()) {
                cam_command_map[id] = "end";
            }
        } else if (command.at(0) == STREAM_STOP) {
            auto parsed = parse_cmd(command);
            int id = -400;
            if (parsed["id"] == "wr") {
                id = -1;
            } else {
                id = std::stoi(parsed["id"]);
            }

            std::cout << "Stopping camera " << id << std::endl;

            // Remove camera from streaming cams
            if (streaming_cams.find(id) != streaming_cams.end()) {
                streaming_cams.erase(id);
            }

            // If the camera is not being used locally, stop it entirely to save resources
            if (local_cams.find(id) == local_cams.end()) {
                cam_command_map[id] = "end";
            }
        }

        // Miniscule efficiency improvement - we sleep less here in case there are more commands
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void begin_handler_loop() {
    // Start the handler loop in a new thread
    threads.insert(std::pair<int, std::thread>(-1, std::thread(handler_loop)));
}

void clean_command_handler(void) {
    // Signal end for all threads
    running = false;

    // Close all sockets
    local_publisher.close();
    subscriber.close();

    // Terminate the context
    context.close();

    // Join all threads (that should have been terminated already)
    // TODO this is a danger point so come back when more code is written
    // also linked to many danger points :) use timeouts
    for (auto& thread : threads) {
        // FIXME we have to kill the stream processes STUPID
        thread.second.join();
    }
}



/**
 * @file command_handler.cpp
 * @author William Streck
 * @brief Command handler implementation, including static functions for use as threads.
 * @version 0.1
 * @date 2024-10-28
 * 
 */

#include "command_handler.hpp"
#include "utilities/communication/streaming/streaming.hpp"
#include <iostream>
#include <thread>
#include <mutex>

static zmq::context_t context; ///< ZMQ context for communication.
static zmq::socket_t local_publisher; ///< ZMQ local publisher socket.
static zmq::socket_t remote_publisher; ///< ZMQ remote publisher socket.
static zmq::socket_t subscriber; ///< ZMQ remote subscriber socket.

static std::map<int, Camera> cameras; ///< Vector of camera objects. Used for seeing if a camera is on.
static std::vector<int> streams; ///< Vector of stream objects. Used for seeing if a stream is on.

static std::map<int, std::string> cam_command_map; ///< Vector of end flags for each thread, regardless of type.

std::map<int, std::thread> threads; ///< Vector of threads for command execution.
std::list<int> threads_end; ///< Vector of threads that have ended.

static std::mutex local_publisher_mutex; ///< Mutex for local publisher socket.
static std::mutex remote_publisher_mutex; ///< Mutex for remote publisher socket.

static bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.

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

    remote_publisher = zmq::socket_t(context, ZMQ_PUB);
    remote_publisher.bind(ZMQ_REMOTE_PUB);
    
    subscriber = zmq::socket_t(context, ZMQ_SUB);
    subscriber.connect(ZMQ_LOCAL_REC);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    // Start thread handler loop
    begin_handler_loop();
}

static void local_camera_start(std::string command, int tmap_index) {
    // Parse the command
    auto parsed = parse_cmd(command);
    auto quality = std::stoi(parsed["qu"]);
    auto camera_id = std::stoi(parsed["id"]);

    if (cameras.find(camera_id) != cameras.end()) {
        // Camera is already on
        // TODO we should send a string message back to the client
        threads_end.push_back(tmap_index); // Clean thread
        return;
    }

    auto camera = Camera();
    auto set = settings();
    set.device_index = camera_id;
    if (quality == 0) {
        set.use_preset(lowest);
    } else if (quality == 1) {
        set.use_preset(low);
    } else if (quality == 2) {
        set.use_preset(okay);
    } else if (quality == 3) {
        set.use_preset(medium);
    } else if (quality == 4) {
        set.use_preset(high);
    } else if (quality == 5) {
        set.use_preset(highest);
    }

    camera.configure(set);
    std::cout << "Starting camera " << camera_id << std::endl;
    try {
        camera.start();
    } catch(const std::exception& e) {
        // Please note this will be caught and handled inside the while loop - I am avoiding duplicate code
        std::cerr << e.what() << '\n';
    }
    cameras.insert(std::pair<int, Camera>(camera_id, camera));
    while (running) {
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
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue; // don't need to publish an empty frame
        }
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

        // Handle a command if one is present
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            if (cam_command_map[camera_id] == "end") {
                // TODO send a verification message
                std::cout << "Camera " << camera_id << " has been terminated." << std::endl;
                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            }
        }
        // [ ] still might want to sleep here
    }
}

static void stream_camera_start(std::string command, int tmap_index) {
    auto parsed = parse_cmd(command);
    auto quality = std::stoi(parsed["qu"]);
    auto camera_id = std::stoi(parsed["id"]);

    for (int stream: streams) {
        if (stream == camera_id) {
            // Stream is already on
            // TODO we should send a string message back to the client
            return;
        }
    }

    // TODO get options from command
    auto set = settings();
    if (quality == 0) {
        set.use_preset(lowest);
    } else if (quality == 1) {
        set.use_preset(low);
    } else if (quality == 2) {
        set.use_preset(okay);
    } else if (quality == 3) {
        set.use_preset(medium);
    } else if (quality == 4) {
        set.use_preset(high);
    } else if (quality == 5) {
        set.use_preset(highest);
    }

    FILE* pipe = ffmpeg_stream_camera(set, camera_id);
    if (!pipe) {
        // TODO send a message back to the client
        return;
    }

    pid_t ffmpeg_pid = fileno(pipe);

    while (running) {
        // Check for termination of this particular stream
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            if (ffmpeg_pid != -1) {
                // It is completely safe from testing to hard kill the process
                kill(ffmpeg_pid, SIGKILL);
            }
            pclose(pipe);
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (ffmpeg_pid != -1) {
        // It is completely safe from testing to hard kill the process
        kill(ffmpeg_pid, SIGKILL);
    }
    pclose(pipe);
    // FIXME thread clean signal
}

static void handler_loop() {
    int map_counter = 0;
    while (1) {
        // Quickly clean any terminated threads
        for (auto clear: threads_end) {
            threads[clear].join();
            threads.erase(clear);
            threads_end.remove(clear);
            break; // Only clean one at a time so we don't iterate back through
        }

        zmq::message_t message;
        subscriber.recv(&message, ZMQ_NOBLOCK);
        std::string command = std::string(static_cast<char*>(message.data()), message.size());
        std::cout << "Received command: " << command << std::endl;
        if (command.size() < 3) {
            // Invalid command - sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (command.at(0) == LOCAL_START) {
            threads.insert(std::pair<int, std::thread>(map_counter, std::thread(local_camera_start, command, map_counter)));
            std::cout << "Thread " << map_counter << " has been created." << std::endl;
            map_counter++;
        } else if (command.at(1) == STREAM_START) {
            threads.insert(std::pair<int, std::thread>(map_counter, std::thread(stream_camera_start, command, map_counter)));
            map_counter++;
        } else if (command.at(0) == LOCAL_STOP) {
            // TODO ffmpeg process must be annihilated as well
            // If we find a queued command that hasn't been handled, just replace it
            auto id = std::stoi(command.substr(1, 2));
            if (cam_command_map.find(id) != cam_command_map.end()) {
                cam_command_map.erase(id);
            }
            // Confirm the camera is running locally
            if (cameras.find(id) != cameras.end()) {
                // Send the end command and let the thread clean up the camera
                cam_command_map.insert(std::pair<int, std::string>(id, "end"));
            }
            // Otherwise we don't have to do anything
        } else if (command.at(1) == STREAM_STOP) {
            // TODO end ffmpeg process (depending on how I do that)
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
    remote_publisher.close();
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



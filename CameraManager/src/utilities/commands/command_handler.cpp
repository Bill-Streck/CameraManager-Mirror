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
#include <iostream>
#include <thread>
#include <mutex>
#ifndef SIGKILL
    #include <signal.h>
#endif

static zmq::context_t context; ///< ZMQ context for communication.
static zmq::socket_t local_publisher; ///< ZMQ local publisher socket.

static std::map<int, Camera> cameras; ///< Vector of camera objects. Used for seeing if a camera is on.
static std::map<int, std::string> local_cams; ///< Map of cameras that are being used locally. Used to manage ZMQ workflow.
static std::map<int, std::string> streaming_cams; ///< Map of cameras that are streaming. Used for ffmpeg process management.

static std::map<int, std::string> cam_command_map; ///< Vector of end flags for each thread, regardless of type.

static std::map<int, std::thread> threads; ///< Vector of threads for command execution.
static std::list<int> threads_end; ///< Vector of threads that have ended.

static std::mutex local_publisher_mutex; ///< Mutex for local publisher socket.

static bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.

int map_counter = 0; ///< Counter for the thread map. Respectfully, if you manage to make 2^31 threads, I will be impressed.

class ImgPublisher : public rclcpp::Node
{
    public:
        ImgPublisher()
        : Node("img_publisher")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>
            ("image_topic", 10);
        }

        void publish_image(sensor_msgs::msg::Image msg) {
            publisher_->publish(msg);
        }

    private:
        // Hold a list of publishers by numeric index
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher_;
};

static std::shared_ptr<ImgPublisher> local_ROS_publisher;

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
    // ZMQ
    context = zmq::context_t(1);

    local_publisher = zmq::socket_t(context, ZMQ_PUB);
    local_publisher.bind(ZMQ_LOCAL_PUB);

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

    if (cameras.find(camera_id) != cameras.end() || camera_id == -400 || camera_id > MAX_CAMERA_ID) {
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
            if (pipe == nullptr) {
                std::cerr << "Error starting ffmpeg process for camera " << camera_id << std::endl;
                // TODO send a verification message
                cam_command_map[camera_id] = "end";
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

        // Check if we should be running local ZMQ comms
        if (!cam_local && local_cams.find(camera_id) != local_cams.end()) {
            cam_local = true;
        } else if (cam_local && local_cams.find(camera_id) == local_cams.end()) {
            cam_local = false;
        }
        
        // Get the frame
        auto frame = camera.get_current_frame();

        // Get the timestamp (after frame should be most accurate)
        auto now = std::chrono::system_clock::now();
        auto now_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
        auto now_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - now_seconds);
        auto ts_seconds = static_cast<uint32_t>(now_seconds.time_since_epoch().count());
        auto ts_nanoseconds = static_cast<uint32_t>(now_nanoseconds.count());

        if (frame.empty()) {
            std::cout << "Camera " << camera_id << " has no frame." << std::endl;
            try {
                camera.start();
            } catch(const std::exception& e) {
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
            std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_FAILURE_RETRY));
            continue; // don't need to publish an empty frame
        }

        // Stream if applicable
        if (cam_streaming && pipe != nullptr) {
            fwrite(frame.data, 1, frame.total() * frame.elemSize(), pipe);
        }

        // Publish the frame locally if applicable
        if (cam_local) {
            // [ ] if applicable, remove ZMQ
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

            // Put the image into a ROS2 message
            auto msg = sensor_msgs::msg::Image();
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = "bgr8"; // pixel encoding
            msg.step = frame.step;
            // Don't worry about endian leave it default
            msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.total() * frame.elemSize());

            // Information in header
            msg.header.stamp.sec = ts_seconds;
            msg.header.stamp.nanosec = ts_nanoseconds;
            msg.header.frame_id = std::to_string(camera_id);

            // Publish the message
            local_ROS_publisher->publish_image(msg);
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
            // TODO attribute handlers - should work through the camera object directly
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
            std::cout << "Thread " << clear << " has been cleaned." << std::endl;
        }

        std::string command = get_command();
        if (command.size() == 0) {
            // Command doesn't exist
            std::this_thread::sleep_for(std::chrono::milliseconds(HANDLER_NO_MESSAGE_SLEEP));
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
        // TODO attribute, force restart

        // Miniscule efficiency improvement - we sleep less here in case there are more commands
        std::this_thread::sleep_for(std::chrono::milliseconds(HANDLER_MESSAGE_SLEEP));
    }
}

void begin_handler_loop() {
    // Start the handler loop in a new thread
    threads.insert(std::pair<int, std::thread>(-1, std::thread(handler_loop)));
    local_ROS_publisher = std::make_shared<ImgPublisher>();
    threads.insert(std::pair<int, std::thread>(-2, std::thread([]() {
        rclcpp::spin(local_ROS_publisher);
    })));
}

void clean_command_handler(void) {
    // Signal end for all threads
    running = false;

    // Close the publisher
    local_publisher.close();

    // Terminate the context
    context.close();

    // Join all threads (that should have been terminated already)
    for (auto& thread : threads) {
        // ffmpeg processes die with this application
        thread.second.join();
    }
}



/**
 * @file camera_thread.cpp
 * @author William Streck
 * @brief Defines the camera thread function and relevant static functions.
 * @version 0.1
 * @date 2025-02-05
 * 
 */

#include "CameraManagerNode.hpp"
#include "camera_thread.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <thread>

extern shared_ptr<CameraManager> camera_manager_node;

mutex publisher_mutex; ///< Mutex for local publisher socket.
list<int> threads_end; ///< List of threads to end.
map<int, Camera> cameras; ///< Map of camera objects. Used for seeing if a camera is on.
bool running = true; ///< Running flag for the handler loop. Doesn't need to be atomic.
map<int, string> local_cams; ///< Map of cameras that are being used locally.
map<int, string> streaming_cams; ///< Map of cameras that are streaming. Used for ffmpeg process management.
map<int, map<string, string>> cam_command_map; ///< Map of end flags for each thread, regardless of type.

clarity preset_from_quality(int quality) {
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
        return highish;
    } else if (quality == 8) {
        return high;
    } else if (quality == 9) {
        return higher;
    } else if (quality == 10) {
        return highest;
    }

    // fallback
    return okay;
}

int find_cam_id(map<string, string> parsed) {
    int camera_id = CAMERA_ID_FAIL;
    if (parsed[INDEX_ID] == WRIST_ID) {
        camera_id = -1;
    } else {
        try {
            camera_id = stoi(parsed[INDEX_ID]);
        } catch (const exception& e) {
            return CAMERA_ID_FAIL;
        }
    }

    return camera_id;
}

void logi_cam_thread(map<string, string> parsed, int tmap_index) {
    // Pull the important values from the command
    auto quality = stoi(parsed[INDEX_QUALITY]);
    int camera_id = find_cam_id(parsed);

    if (camera_id == CAMERA_ID_FAIL) {
        // Camera is already on
        // TODO we should send a string message back to the client
        threads_end.push_back(tmap_index); // Clean thread
        return;
    }

    // TODO enable having two different settings for streaming and local
    auto camera = Camera();
    auto sett = settings();
    sett.device_index = camera_id;
    auto preset = preset_from_quality(quality);
    sett.use_preset(preset);

    camera.configure(sett);

    try {
        camera.start();
    } catch(const exception& e) {
        // Please note this will be caught and handled inside the while loop - I am avoiding duplicate code
    }

    // Get basic camera variables ready
    cameras[camera_id] = camera;
    auto cam_streaming = false;
    auto cam_local = false;
    // Encoder encoder = Encoder();
    FILE* pipe = nullptr; // Pipe for ffmpeg process

    // Capture loop
    while (running) {
        // Check if we should be streaming or should close a stream
        if (!cam_streaming && streaming_cams.find(camera_id) != streaming_cams.end()) {
            // Create the pipe
            pipe = ffmpeg_stream_camera(sett, camera_id);
            if (pipe == nullptr) {
                // TODO send a message back to the client over the debug channel
                RCLCPP_ERROR(rclcpp::get_logger("CameraManager"), "Failed to start ffmpeg stream");

                // Erase from streaming cams for now
                if (streaming_cams.find(camera_id) != streaming_cams.end()) {
                    streaming_cams.erase(camera_id);
                }
            }

            cam_streaming = true;
        } else if (cam_streaming && streaming_cams.find(camera_id) == streaming_cams.end()) {
            cam_streaming = false;

            // Stop the ffmpeg process
            if (streaming_cams.find(camera_id) != streaming_cams.end()) {
                kill(fileno(pipe), SIGKILL);
                pclose(pipe);
                streaming_cams.erase(camera_id);
                pipe = nullptr;
            }
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

        // TODO very important - resolution addressing
        // After we address containing separate resolutions for local and streaming and live updates,
        // we need to make sure to update the ffmpeg process with the new resolution
        // We also need to implement resizing (just basic opencv resize) for whichever is smaller

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
            if (camera_id >= 0) {
                // HACK magic number just put this in the header
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
        if (cam_streaming) {
            // TODO all the pipe safety stuff
            fwrite(frame.data, 1, frame.total() * frame.elemSize(), pipe);
            // Luckily, we shouldn't need to do anything else now!! :)
        }

        // Handle a command if one is present
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            // TODO still need to appropriately handle resolution changes and restart the ffmpeg process if applicable
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
            } else {
                // Fallback just don't care
                cam_command_map.erase(camera_id);
            }
        }
    }
}
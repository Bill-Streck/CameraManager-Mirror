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
map<int, map<string, int>> cam_command_map; ///< Map of end flags for each thread, regardless of type.

clarity preset_from_quality(int quality) {
    if (0 <= quality && quality <= highest) {
        return static_cast<clarity>(quality);
    }

    // fallback
    return okay;
}

void logi_cam_thread(map<string, int> parsed, int tmap_index) {
    // Pull the important values from the command
    auto quality = parsed[INDEX_QUALITY];
    int camera_id = parsed[INDEX_ID];

    if (camera_id == CAMERA_ID_FAIL) {
        // Camera is already on
        threads_end.push_back(tmap_index); // Clean thread
        return;
    }

    // Settings and camera structure
    auto camera = Camera();
    auto local_sett = settings();
    local_sett.device_index = camera_id;
    auto preset = preset_from_quality(quality);
    local_sett.use_preset(preset);
    auto stream_sett = local_sett.deep_copy(); // Deep copy for streaming settings

    // Internal settings that define how the camera is actually running
    int internal_fps = local_sett.fps; // Used for frame filtering
    int internal_width = local_sett.width;
    int internal_height = local_sett.height;

    // local and streaming-specific settings
    int local_width = local_sett.width;
    int local_height = local_sett.height;
    // Local fps will not be filtered as of now
    // Streaming fps will always have filtering permitted for multiples of less than 5. Any higher is moreso pointless.
    int local_fps = local_sett.fps;
    int stream_fps = local_sett.fps;
    int stream_width = local_width;
    int stream_height = local_height;

    camera.configure(local_sett);

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
            pipe = ffmpeg_stream_camera(stream_sett, camera_id);
            if (pipe == nullptr) {
                RCLCPP_ERROR(camera_manager_node->get_logger(), "ffmpeg failed - advise stop and restart camera");

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
                kill(fileno(pipe), SIGTERM); // SIGTERM - graceful shutdown, may help with rtsp running
                pclose(pipe);
                streaming_cams.erase(camera_id);
                pipe = nullptr;
            }
        }

        // Check if we should be running local ros2 messages
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
            } catch(const exception& e) {}
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
            auto local_frame = frame; // Rename for scoping purposes

            // If streaming and local have different resolutions...
            // AND the local resolution is smaller than the stream resolution...
            // We want to resize the local frame.
            // Note this is a pretty rare case and will only come up in testing
            if (local_width != stream_width && local_width < stream_width) {
                local_frame = cv::Mat(); // Now we want to allocate this
                cv::resize(frame, local_frame, cv::Size(local_width, local_height));
            }
            msg.height = local_frame.rows;
            msg.width = local_frame.cols;
            msg.encoding = "bgr8"; // pixel encoding
            msg.step = local_frame.step;
            // Don't worry about endian leave it default
            msg.data = vector<uint8_t>(local_frame.data, local_frame.data + local_frame.total() * local_frame.elemSize());

            // Information in header
            msg.header.stamp.sec = ts_seconds;
            msg.header.stamp.nanosec = ts_nanoseconds;
            msg.header.frame_id = to_string(camera_id);

            // metadata
            auto meta_msg = robot_interfaces::msg::ImageMetadata();
            meta_msg.im_height = local_frame.rows;
            meta_msg.im_width = local_frame.cols;
            if (camera_id >= 0) {
                meta_msg.sensor_height = SENSOR_HEIGHT_MM;
                meta_msg.foc_len_mm = FOCAL_LENGTH_MM;
            } else {
                // Nobody should be listenting to this for the realsense camera
                meta_msg.sensor_height = 0;
                meta_msg.foc_len_mm = 0;
            }

            // Publish the messages
            camera_manager_node->publish_image(msg);
            camera_manager_node->publish_img_meta(meta_msg);
        }

        // Stream if applicable
        if (cam_streaming) {
            // If streaming and local have different resolutions...
            // AND the stream resolution is smaller than the local resolution...
            // We want to resize the stream frame.
            // Unlike the other, this might be pretty common during autonomy (1080p -> 180p or 90p)
            auto stream_frame = frame; // Rename for scoping purposes
            if (stream_width != local_width && stream_width < local_width) {
                stream_frame = cv::Mat(); // Now we want to allocate this
                cv::resize(frame, stream_frame, cv::Size(stream_width, stream_height));
            }

            // TODO all the pipe safety stuff
            fwrite(stream_frame.data, 1, stream_frame.total() * stream_frame.elemSize(), pipe);
            // Luckily, we shouldn't need to do anything else now!! :)
        }

        // Handle a command if one is present
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            auto cmd = cam_command_map[camera_id];

            if (cmd[AUX_INDEX_BASE] == COMMAND_MAP_END) {
                // End command
                camera.stop_all();
                cam_command_map.erase(camera_id);
                cameras.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;

            } else if (cmd[INDEX_MODE] == ATTRIBUTE_MODIFY) {
                // Handle attribute modification
                auto attribute = cmd[INDEX_ATTRIBUTE];
                auto value = cmd[INDEX_AT_VALUE];
                
                if (attribute == ATTR_STREAM_RESOLUTION && cam_streaming) {
                    // Immediately restart the ffmpeg process before another frame can even exist
                    kill(fileno(pipe), SIGTERM); // SIGTERM - graceful shutdown, may help with rtsp running
                    pclose(pipe);

                    // Set only the new resolution and not fps
                    int placeholder_fps = stream_fps;
                    stream_sett.use_preset(preset_from_quality(value));
                    stream_width = stream_sett.width;
                    stream_height = stream_sett.height;
                    stream_sett.fps = placeholder_fps;
                    // Don't change fps for now - there is another command for that
                    pipe = ffmpeg_stream_camera(stream_sett, camera_id);
                    if (pipe == nullptr) {
                        RCLCPP_ERROR(camera_manager_node->get_logger(), "ffmpeg failed - advise stop and restart camera");

                        // Erase from streaming cams for now
                        if (streaming_cams.find(camera_id) != streaming_cams.end()) {
                            streaming_cams.erase(camera_id);
                        }
                    }

                    // Check if we need to increase the internal resolution
                    if (internal_width < stream_width) {
                        // Increase internal resolution to the next step
                        internal_width = stream_width;
                        internal_height = stream_height;
                        if (!camera.change_attribute(ATTR_INTERNAL_RES, value)) {
                            RCLCPP_ERROR(camera_manager_node->get_logger(), "Failed to change internal resolution to %d", value);
                        }
                    }
                } else if (attribute == ATTR_LOCAL_RESOLUTION && cam_local) {
                    // Same as above but without the ffmpeg process

                    // Set only the new resolution and not fps
                    int placeholder_fps = local_fps;
                    local_sett.use_preset(preset_from_quality(value));
                    local_width = local_sett.width;
                    local_height = local_sett.height;
                    local_sett.fps = placeholder_fps;
                    // Don't change fps for now - there is another command for that

                    // Check if we need to increase the internal resolution
                    if (internal_width < local_width) {
                        // Increase internal resolution to the next step
                        internal_width = local_width;
                        internal_height = local_height;
                        if (!camera.change_attribute(ATTR_INTERNAL_RES, value)) {
                            RCLCPP_ERROR(camera_manager_node->get_logger(), "Failed to change internal resolution to %d", value);
                        }
                    }
                } else if (!camera.change_attribute(attribute, value)) {
                    RCLCPP_WARN(camera_manager_node->get_logger(), "Failed to change attribute %d to %d", attribute, value);
                }

                // Remove the command from the map
                cam_command_map.erase(camera_id);

            } else {
                // Fallback just don't care
                RCLCPP_WARN(camera_manager_node->get_logger(), "Unknown command %d", cmd[AUX_INDEX_BASE]);
                cam_command_map.erase(camera_id);
            }
        }
    }
}
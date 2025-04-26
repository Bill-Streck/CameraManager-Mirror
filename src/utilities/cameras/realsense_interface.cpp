/**
 * @file realsense_interface.cpp
 * @author William Streck
 * @brief Realsense interface functions for the Camera Manager.
 * @version 0.1
 * @date 2025-02-22
 * 
 */

#include "realsense_interface.hpp"
#include "CameraManagerNode.hpp"

static queue<sensor_msgs::msg::Image> realsense_frame_queue;

void CameraManager::realsense_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    // If and only if we are using the wrist camera, push the frame to the queue
    if (cameras.find(-1) != cameras.end()) {
        realsense_frame_queue.push(*msg);
    } else {
        // ensure the queue is cleared if we are not watching the camera
        while (!realsense_frame_queue.empty()) {
            realsense_frame_queue.pop();
        }
    }
}

void realsense_cam_thread(map<string, int> parsed, int tmap_index) {
    // verify the camera ID
    if (parsed[INDEX_ID] != WRIST_ID) {
        threads_end.push_back(tmap_index);
        return;
    }

    // Get the important stuff - we WON'T be using a camera object - just receiving images
    auto quality = parsed[INDEX_QUALITY];
    auto preset = preset_from_quality(quality);

    // Settings are used specifically for height and width resizing
    auto sett = settings();
    sett.use_preset(preset);
    auto width = sett.width;
    auto height = sett.height;
    // FIXME this has nothing to do with anything bro what are you doing.
    auto fps = sett.fps;
    int64_t sleep_time_ms = 1000 / (fps + FPS_SLEEP_BUFF); // Time to sleep in milliseconds.
    auto sleep_t = chrono::milliseconds(sleep_time_ms);
    auto cam_local = false;
    auto cam_streaming = false;
    const int camera_id = -1; // Wrist ID
    FILE* pipe = nullptr;

    while (true) {
        // Grab any available frames
        while (realsense_frame_queue.size() > 0) {
            // Check if we should be running local comms
            if (!cam_local && local_cams.find(camera_id) != local_cams.end()) {
                cam_local = true;
            } else if (cam_local && local_cams.find(camera_id) == local_cams.end()) {
                cam_local = false;
            }

            // Check if we should be streaming or should close a stream
            if (!cam_streaming && streaming_cams.find(camera_id) != streaming_cams.end()) {
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
                if (pipe != nullptr) {
                    kill(fileno(pipe), SIGKILL);
                    pclose(pipe);
                    pipe = nullptr;
                }
            }

            // Get the frame
            auto msg = realsense_frame_queue.front();
            realsense_frame_queue.pop();

            // HACK temporarily addresses streaming resolution
            // Does not help with local. Need to implement that along with the rest of the cams.

            // Check if we need to change the size
            // HACK we and this assuming aspect is not constant
            // Need to go in here and just change whichever edges are possible
            if (msg.width > width && msg.height > height) {
                auto old_msg = msg;
                msg = sensor_msgs::msg::Image();
                msg.height = height;
                msg.width = width;
                msg.encoding = old_msg.encoding; // pixel encoding
                msg.step = old_msg.step;
                // Resize the image
                cv::Mat frame(msg.height, msg.width, CV_8UC3, (void*)msg.data.data());
                cv::resize(frame, frame, cv::Size(width, height));
                msg.data = vector<uint8_t>(frame.data, frame.data + frame.total() * frame.elemSize());
                msg.header.stamp = old_msg.header.stamp;
                // We can just let old_msg go out of scope - the original msg will get deallocated when looping
            }

            if (cam_local) {
                // Steal the ROS2 message - NOT SUPPORTING RESIZING OR METADATA
                msg.header.frame_id = WRIST_ID;
                camera_manager_node->publish_image(msg);
            }

            if (cam_streaming && (pipe != nullptr)) {
                // This is where we may need to do some resizing - DOWNSCALE ONLY
                cv::Mat frame(msg.height, msg.width, CV_8UC3, (void*)msg.data.data());
                // FIXME resizing infrastructure not chillin!!!!!!!!
                fwrite(frame.data, 1, frame.total() * frame.elemSize(), pipe);
                // Just like the other cams, we don't have to do anything else now!!
            }
        }

        // No commands except end
        if (cam_command_map.find(camera_id) != cam_command_map.end()) {
            auto cmd = cam_command_map[camera_id];
            if (cmd[AUX_INDEX_BASE] == COMMAND_MAP_END) {
                // TODO send a verification message

                // Stop listening to the realsense. Let the realsense package do what it wants.
                cam_command_map.erase(camera_id);
                threads_end.push_back(tmap_index); // Must occur LAST
                return;
            } else {
                // No other commands exist
                cam_command_map.erase(camera_id);
            }
        }

        // Sleep for the appropriate time
        this_thread::sleep_for(sleep_t);
    }
}
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
    }
}

void realsense_cam_thread(map<string, string> parsed, int tmap_index) {
    // verify the camera ID
    if (parsed[INDEX_ID] != WRIST_ID) {
        threads_end.push_back(tmap_index);
        return;
    }

    // Get the important stuff - we WON'T be using a camera object - just receiving images
    auto quality = stoi(parsed[INDEX_QUALITY]);
    auto preset = preset_from_quality(quality);

    // Settings are used specifically for height and width resizing
    auto set = settings();
    set.use_preset(preset);
    auto width = set.width;
    auto height = set.height;
    // FIXME this has nothing to do with anything bro what are you doing.
    auto fps = set.fps;
    int64_t sleep_time_ms = 1000 / (fps + FPS_SLEEP_BUFF); // Time to sleep in milliseconds.
    auto sleep_t = chrono::milliseconds(sleep_time_ms);
    auto cam_local = false;
    auto cam_streaming = false;
    const int camera_id = -1; // Wrist ID
    FILE* pipe = nullptr;

    // Preload the streaming things
    int stream_port = stream_port_from_camera_id(camera_id);
    int local_port = local_port_from_camera_id(camera_id);
    sockaddr_in broadcast_address;
    broadcast_address.sin_family = AF_INET;
    broadcast_address.sin_addr.s_addr = inet_addr(BROADCAST_IP_ADDR);
    broadcast_address.sin_port = htons(stream_port);
    int stream_sockfd = initialize_stream_socket();
    int local_sockfd = initialize_local_socket(local_port);

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

            // Get the frame
            auto msg = realsense_frame_queue.front();
            realsense_frame_queue.pop();

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

                ssize_t recv_len = 0L;

                do {
                    // TODO buffer size is a guess and way oversized
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
                } while (recv_len >= MPEGTS_PACKET_MAX);
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
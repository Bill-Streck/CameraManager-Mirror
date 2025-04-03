/**
 * @file CameraManager.cpp
 * @author William Streck
 * @brief CameraManager main execution path.
 * @version 0.1
 * @date 2024-10-24
 * 
 * @note Advised to keep sparse.
 */

#include "CM_include.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::shared_ptr<CameraManager> camera_manager_node; 

CameraManager::CameraManager() : Node("camera_manager") {
    // Confirm topic parameters
    this->declare_parameter(CM_SUB_TOP_PARAM, CM_SUB_TOPIC);
    this->declare_parameter(CM_PUB_TOP_PARAM, CM_PUB_TOPIC);
    this->declare_parameter(CM_IMG_TOP_PARAM, CM_IMAGE_TOPIC);
    this->declare_parameter(CM_MET_TOP_PARAM, CM_METADATA_TOPIC);
    auto sub_topic = this->get_parameter(CM_SUB_TOP_PARAM).as_string();
    auto pub_topic = this->get_parameter(CM_PUB_TOP_PARAM).as_string();
    auto img_topic = this->get_parameter(CM_IMG_TOP_PARAM).as_string();
    auto met_topic = this->get_parameter(CM_MET_TOP_PARAM).as_string();

    // Subscriber and publishers
    {
        subscription_ = this->create_subscription<robot_interfaces::msg::CameraManagerCommand>
            (sub_topic, 10, std::bind(&CameraManager::command_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt32>
            (pub_topic, 10);

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
            (img_topic, 10);

        metadata_publisher_ = this->create_publisher<robot_interfaces::msg::ImageMetadata>
            (met_topic, 10);
    }

    // TODO prestarts will be same function at some point so just pull these together or just change the functions called
    // Camera prestarts
    {
        this->declare_parameter(
            CAM_PRESTART_PARAM, 
            vector<int64_t>());

        this->declare_parameter(
            CAM_PRST_QUAL_PARAM,
            vector<int64_t>());

        auto prestarts = this->get_parameter(CAM_PRESTART_PARAM).as_integer_array();
        auto prestart_quality = this->get_parameter(CAM_PRST_QUAL_PARAM).as_integer_array();

        if (prestarts.size() > 0) {
            // We need to prestart some cameras
            prestart_cameras(prestarts, prestart_quality);
        }
    }

    // Cam stream prestarts
    {
        this->declare_parameter(
            CAM_STREAM_PRESTART_PARAM, 
            vector<int64_t>());

        this->declare_parameter(
            CAM_STREAM_PRST_QUAL_PARAM,
            vector<int64_t>());

        auto prestarts = this->get_parameter(CAM_STREAM_PRESTART_PARAM).as_integer_array();
        auto prestart_quality = this->get_parameter(CAM_STREAM_PRST_QUAL_PARAM).as_integer_array();

        if (prestarts.size() > 0) {
            // We need to prestart some cameras
            prestart_stream_cameras(prestarts, prestart_quality);
        }
    }
}

void CameraManager::publish_debug(uint32_t data) {
    auto message = std_msgs::msg::UInt32();
    message.data = data;
    publisher_->publish(message);
}

void CameraManager::command_callback(const robot_interfaces::msg::CameraManagerCommand::SharedPtr msg) const {
    handle_command(msg);
}

void CameraManager::publish_image(sensor_msgs::msg::Image msg) {
    image_publisher_->publish(msg);
}

void CameraManager::publish_img_meta(robot_interfaces::msg::ImageMetadata msg) {
    metadata_publisher_->publish(msg);
}

static void CM_shutdown(void) {
    rclcpp::shutdown();
    clean_command_handler();
}

void signal_handler(int signum) {
    CM_shutdown();

    while (true) {
        exit(signum);
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    // Start ROS2 BEFORE the command handler. If we have a test node, launch it on a thread
    rclcpp::init(argc, argv);

    // Initialize any utilites with no ROS dependencies
    init_command_handler();

    camera_manager_node = std::make_shared<CameraManager>();
    rclcpp::spin(camera_manager_node);

    // Clean up - signal handler will take it if this doesn't
    CM_shutdown();
    
    return 0;
}

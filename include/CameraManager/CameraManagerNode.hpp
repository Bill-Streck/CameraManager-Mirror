/**
 * @file CameraManagerNode.hpp
 * @author William Streck
 * @brief Declares the CameraManager node and its global pointer.
 * @version 0.1
 * @date 2024-11-21
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "camera_metadata.hpp"

#define CM_SUB_TOPIC "camera_manager" ///< Main command listener topic.
#define CM_PUB_TOPIC "camera_manager_debug" ///< Debug publisher topic.
#define CM_IMAGE_TOPIC "image_topic" ///< Image publisher topic (local on rover).
#define CM_METADATA_TOPIC "CM_cam_meta" ///< Metadata topic for YOLO.

/**
 * @brief The CameraManager class is the main ROS2 node for the CameraManager package.
 * 
 */
class CameraManager : public rclcpp::Node
{
    public:
        CameraManager();

        /**
         * @brief Publishes a debug message to the camera_manager_debug topic.
         * 
         * @param data The data to publish.
         */
        void publish_debug(uint32_t data);

        /**
         * @brief Publishes an image message to the appropriate topic.
         * 
         * @param msg Image to publish
         */
        void publish_image(sensor_msgs::msg::Image msg);

        /**
         * @brief Publishes image metadata to the appropriate topic.
         * 
         * @param msg Metadata to publish.
         */
        void publish_img_meta(camera_manager::msg::ImageMetadata msg);

    private:
        /**
         * @brief Processes a CameraManager input message command
         * 
         * @param msg 
         */
        void command_callback(const std_msgs::msg::UInt32::SharedPtr msg) const;

        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_; ///< CameraManager command subscription.
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_; ///< CameraManager debug response publisher.
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; ///< CameraManager image publisher.
        rclcpp::Publisher<camera_manager::msg::ImageMetadata>::SharedPtr metadata_publisher_; ///< CameraManager metadata publisher. Specifically used for YOLO data.
};

extern std::shared_ptr<CameraManager> camera_manager_node;
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
#include "robot_interfaces/msg/camera_manager_command.hpp"
#include "camera_metadata.hpp"
#include "realsense_interface.hpp"
#include "sensor_msgs/msg/image.hpp"

#define CM_SUB_TOP_PARAM "cm_sub_topic" ///< Subscription topic param.
#define CM_PUB_TOP_PARAM "cm_pub_topic" ///< Debug publisher topic param.
#define CM_IMG_TOP_PARAM "cm_image_topic" ///< Image topic param.
#define CM_MET_TOP_PARAM "cm_meta_topic" ///< Metadata publisher topic param.
#define CAM_PRESTART_PARAM "cameras_prestart" ///< Camera prestart parameter as list of camera numbers.
#define CAM_PRST_QUAL_PARAM "cam_prestart_qual" ///< Prestart quality parameter as list of numbers.

#define CM_SUB_TOPIC "camera_manager" ///< Main command listener topic default.
#define CM_PUB_TOPIC "camera_manager_debug" ///< Debug publisher topic default.
#define CM_IMAGE_TOPIC "image_topic" ///< Image publisher topic (local on rover) default.
#define CM_METADATA_TOPIC "CM_cam_meta" ///< Metadata topic for YOLO default.

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
        void publish_img_meta(robot_interfaces::msg::ImageMetadata msg);

    private:
        /**
         * @brief Processes a CameraManager input message command
         * 
         * @param msg Command to address.
         */
        void command_callback(const robot_interfaces::msg::CameraManagerCommand::SharedPtr msg) const;

        /**
         * @brief Listens for images from the RealSense camera.
         * 
         * @param msg Image received.
         */
        void realsense_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;

        rclcpp::Subscription<robot_interfaces::msg::CameraManagerCommand>::SharedPtr subscription_; ///< CameraManager command subscription.
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_; ///< CameraManager debug response publisher.
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; ///< CameraManager image publisher.
        rclcpp::Publisher<robot_interfaces::msg::ImageMetadata>::SharedPtr metadata_publisher_; ///< CameraManager metadata publisher. Specifically used for YOLO data.
};

extern std::shared_ptr<CameraManager> camera_manager_node;
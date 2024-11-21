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

#define CM_SUB_TOPIC "camera_manager"
#define CM_PUB_TOPIC "camera_manager_debug"

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

    private:
        void command_callback(const std_msgs::msg::UInt32::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_; ///< Camera command subscription.
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_; ///< Camera debug response publisher;
};

extern std::shared_ptr<CameraManager> camera_manager;
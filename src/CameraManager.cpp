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
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::shared_ptr<CameraManager> camera_manager_node; 

// FIXME parameter topic names
CameraManager::CameraManager() : Node("camera_manager") {
    subscription_ = this->create_subscription<std_msgs::msg::UInt32>
        (CM_SUB_TOPIC, 10, std::bind(&CameraManager::command_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::UInt32>
        (CM_PUB_TOPIC, 10);

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
        (CM_IMAGE_TOPIC, 10);

    metadata_publisher_ = this->create_publisher<robot_interfaces::msg::ImageMetadata>
        (CM_METADATA_TOPIC, 10);

    this->declare_parameter(
        CAM_PRESTART_PARAM, 
        std::vector<int64_t>());

    // TODO eval as array later???
    this->declare_parameter(
        CAM_PRST_QUAL_PARAM,
        -1
    );

    auto prestarts = this->get_parameter(CAM_PRESTART_PARAM).as_integer_array();
    auto prestart_quality = this->get_parameter(CAM_PRST_QUAL_PARAM).as_int();

    if (prestarts.size() > 0) {
        // We need to prestart some cameras
        std::cout << "weeee" << std::endl;
        prestart_cameras(prestarts, prestart_quality);
    }
}

void CameraManager::publish_debug(uint32_t data) {
    auto message = std_msgs::msg::UInt32();
    message.data = data;
    publisher_->publish(message);
}

void CameraManager::command_callback(const std_msgs::msg::UInt32::SharedPtr msg) const {
    handle_command(msg->data);
}

void CameraManager::publish_image(sensor_msgs::msg::Image msg) {
    image_publisher_->publish(msg);
}

void CameraManager::publish_img_meta(robot_interfaces::msg::ImageMetadata msg) {
    metadata_publisher_->publish(msg);
}

class TestPub : public rclcpp::Node
{
    public:
        TestPub()
        : Node("test_pub")
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt32>(CM_SUB_TOPIC, 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&TestPub::timer_callback, this));
        }

    private:
        uint32_t count = 0;
        uint32_t commands[6] = {
            // Start camera 7 local
            // 0b000'00010'00111'000'0000'0000'0000'0000,
            0b000'00100'00011'000'0000'0000'0000'0000, // 3 local
            0b001'00100'00011'000'0000'0000'0000'0000, // 3 stream
            // 21 local
            // 0b000'00100'10101'000'0000'0000'0000'0000,
            // Start camera 8 local
            // 0b000'00010'01000'000'0000'0000'0000'0000,
            // 0b000'00100'00001'000'0000'0000'0000'0000, // 1 local
            // end camera 1 local
            0xFFFFFFFF,
            // Stream camera 7
            // 0b001'00010'00111'000'0000'0000'0000'0000,
            0xFFFFFFFF,
            // end camera 3 local
            0xFFFFFFFF,
            // end camera 3 stream after delay
            0xFFFFFFFF
        };
        void timer_callback()
        {
            auto message = std_msgs::msg::UInt32();
            if (count < 6) {
                message.data = commands[count];
                count++;
                publisher_->publish(message);
            } else {
                // message.data = 0xFFFFFFFF;
                // slowly increase brightness of camera 1 by 10
                message.data = 0b101'00001'0000'0000'0000'0000'0000'1010;
            }
            // publisher_->publish(message);
        }
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

class TestSub : public rclcpp::Node
{
    public:
        TestSub()
        : Node("test_sub")
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                CM_IMAGE_TOPIC, 10, std::bind(&TestSub::command_callback, this, _1));

            debug_listener_ = this->create_subscription<std_msgs::msg::UInt32>(
                CM_PUB_TOPIC, 10, std::bind(&TestSub::debug_callback, this, _1));
        }

    private:
        void command_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            cv::Mat frame(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
            auto imname = "Frame" + msg->header.frame_id + "fromROS";
            try {
                cv::imshow(imname, frame);
                cv::waitKey(1);
            } catch (const cv::Exception& e) {
                // no screen present
                return;
            }
        }

        void debug_callback(const std_msgs::msg::UInt32::SharedPtr msg) const
        {
            std::cout << "Received debug message: " << msg->data << std::endl;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr debug_listener_;
};

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

    // Need to dispatch a thread for one of these (doesn't matter which)
    // auto t = std::thread([]() {
    //     rclcpp::spin(std::make_shared<TestPub>());
    // });
    auto t2 = std::thread([]() {
        rclcpp::spin(std::make_shared<TestSub>());
    });

    camera_manager_node = std::make_shared<CameraManager>();
    rclcpp::spin(camera_manager_node);

    // FIXME we actually do need a way down here
    // Clean up ROS2 and other utilities
    rclcpp::shutdown();
    // t.join();
    // t2.join();
    clean_command_handler();
    
    return 0;
}

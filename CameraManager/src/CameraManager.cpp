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
#include "H264Encoder.hpp"
#include "H264Decoder.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace Codec;
using namespace std::chrono_literals;
using std::placeholders::_1;

class CameraManager : public rclcpp::Node
{
    public:
        CameraManager()
        : Node("camera_manager")
        {
            subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
                "camera_manager", 10, std::bind(&CameraManager::command_callback, this, _1));
        }

    private:
        void command_callback(const std_msgs::msg::UInt32::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
            // handle_command(msg->data);
        }
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
};

// TODO obviously, this needs removal
class TestPub : public rclcpp::Node
{
    public:
        TestPub()
        : Node("test_pub")
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt32>("camera_manager", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&TestPub::timer_callback, this));
        }

    private:
        uint32_t count = 0;
        void timer_callback()
        {
            auto message = std_msgs::msg::UInt32();
            message.data = count++;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
            publisher_->publish(message);
        }
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

// This is a placeholder as ZMQ is going to be retired
static void display_received_frames() {
    // Initialize zmq context
    zmq::context_t context(1);

    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:6666");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    while (1) {        
        // Receive message (blocking)
        zmq::message_t received;
        subscriber.recv(&received);
        uchar cam_number = *(static_cast<uchar*>(received.data()));
        subscriber.recv(&received); // height
        int height = *(static_cast<int*>(received.data()));
        subscriber.recv(&received); // width
        int width = *(static_cast<int*>(received.data()));
        subscriber.recv(&received); // frame

        // temporarily make placeholder size and type
        cv::Mat frame(height, width, CV_8UC3);
        cv::Mat new_frame(frame.size(), frame.type(), received.data());

        if (cam_number == 1) {
            cv::imshow("Frame1", new_frame);
        } else if (cam_number == 3) {
            cv::imshow("Frame3", new_frame);
        } else {
            cv::imshow("FrameERROR", new_frame);
            std::cout << "Error: Camera number spawned in for some reason." << std::endl;
        }

        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }
}

int main(int argc, char* argv[]) {
    // Initialize any utilites with no ROS dependencies
    init_command_handler();
    auto display_thread = std::thread(display_received_frames);

    // Start ROS2. If we have a test node, launch it on a thread
    rclcpp::init(argc, argv);
    // Need to dispatch a thread for one of these (doesn't matter which)
    auto t = std::thread([]() {
        rclcpp::spin(std::make_shared<TestPub>());
    });
    rclcpp::spin(std::make_shared<CameraManager>());

    // Clean up ROS2 and other utilities
    rclcpp::shutdown();
    t.join();
    clean_command_handler();
    return 0;

    if (0) {
        cv::VideoCapture cap(0); // Acer camera bc why not
        if (!cap.isOpened()) {
            std::cerr << "Error opening camera." << std::endl;
            return -1;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360);

        auto encoder = std::make_unique<H264Encoder>();
        auto decoder = std::make_unique<H264Decoder>();
        uint8_t* wEncodedData = nullptr;
        uint8_t* wDecodedData = nullptr;
        int wPacketSize = 0;
        int cumulated_size = 0;

        while (0) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) {
                std::cerr << "Error capturing frame." << std::endl;
                exit(1);
            }

            cv::imshow("Original", frame);
            // cv::cvtColor(frame, frame, cv::COLOR_BGR2YUV_I420);

            // encode
            wEncodedData = encoder->encode(frame.data);
            if (nullptr != wEncodedData) {
                wPacketSize = encoder->getPacketSize();
            }

            // decode
            wDecodedData = decoder->decode(wEncodedData, wPacketSize);
            cv::Mat decodedFrame(360, 640, CV_8UC3);

            if (nullptr != wEncodedData && nullptr != wDecodedData) {
                decodedFrame.data = (uchar*)wDecodedData;
                // check for overflowing data
                std::cout << "Decoded frame size: " << decodedFrame.total() << std::endl;
                std::cout << "expected size: " << 360*640 << std::endl;
            }

            cv::imshow("Frame", decodedFrame);
            char c = (char) cv::waitKey(25);
            if (c == 27) {
                exit(1);
            }
        }
    }

    return 0;
}

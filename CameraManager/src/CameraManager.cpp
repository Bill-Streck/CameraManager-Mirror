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
            handle_command(msg->data);
        }
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
};

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
        uint32_t commands[6] = {
            // Start camera 7 local
            // 0b000'00010'00111'000'0000'0000'0000'0000,
            0b000'00100'00011'000'0000'0000'0000'0000, // 3 local
            // Start camera 8 local
            // 0b000'00010'01000'000'0000'0000'0000'0000,
            0b000'00100'00001'000'0000'0000'0000'0000, // 1 local
            // [ ] end camera 1 local
            0xFFFFFFFF,
            // Stream camera 7
            // 0b001'00010'00111'000'0000'0000'0000'0000,
            0xFFFFFFFF,
            // [ ] end camera 3 local
            0xFFFFFFFF,
            // [ ] end camera 3 stream after delay
            0xFFFFFFFF
        };
        void timer_callback()
        {
            auto message = std_msgs::msg::UInt32();
            if (count < 6) {
                message.data = commands[count];
                count++;
            } else {
                message.data = 0xFFFFFFFF;
            }
            publisher_->publish(message);
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
                "image_topic", 10, std::bind(&TestSub::command_callback, this, _1));
        }

    private:
        void command_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            cv::Mat frame(msg->height, msg->width, CV_8UC3, (void*)msg->data.data());
            auto imname = "Frame" + msg->header.frame_id + "fromROS";
            cv::imshow(imname, frame);
        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// [ ] remove ZMQ if applicable (remove this regardless)
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

        cv::imshow("Frame" + std::to_string(cam_number) + "in ZMQ", new_frame);

        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }
}

int main(int argc, char* argv[]) {
    // Start ROS2 BEFORE the command handler. If we have a test node, launch it on a thread
    rclcpp::init(argc, argv);

    // Initialize any utilites with no ROS dependencies
    init_command_handler();
    auto display_thread = std::thread(display_received_frames);

    // Need to dispatch a thread for one of these (doesn't matter which)
    auto t = std::thread([]() {
        rclcpp::spin(std::make_shared<TestPub>());
    });
    auto t2 = std::thread([]() {
        rclcpp::spin(std::make_shared<TestSub>());
    });
    rclcpp::spin(std::make_shared<CameraManager>());

    // Clean up ROS2 and other utilities
    rclcpp::shutdown();
    t.join();
    t2.join();
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

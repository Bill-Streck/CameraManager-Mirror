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

using namespace Codec;

int main() {
    // test video compression and decompression with theora
    // TODO get rid of this should not be here in implementation is for testing only
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

    // Initialize zmq context
    zmq::context_t context(1);

    // TODO we won't even be using this when we go ROS node (if we do, which we REALLY should)
    zmq::socket_t global_subscriber(context, ZMQ_SUB);
    global_subscriber.connect(ZMQ_REMOTE_REC);
    global_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages
    zmq::socket_t global_publisher(context, ZMQ_PUB);
    global_publisher.bind(ZMQ_REMOTE_PUB);

    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:6666");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    init_command_handler();

    while (1) {
        // receive global message (temporary)
        // TODO this becomes a callback on ROS
        zmq::message_t global_received;
        global_subscriber.recv(&global_received);
        std::string global_message = std::string(static_cast<char*>(global_received.data()), global_received.size());
        std::cout << "Global message: " << global_message << std::endl;
        post_command(global_message);
        
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
        // FIXME We need an actual exit point (gobally accessible) that command handler can use
    }

    cv::destroyAllWindows();
    // clean_command_handler(); // TODO make timeoutable
    std::thread t1(clean_command_handler);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (t1.joinable()) {
        t1.join();
    }

    return 0;
}

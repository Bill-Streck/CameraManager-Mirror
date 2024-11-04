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
#include "utilities/communication/compression/AVIF_ctl.hpp"

int main() {
    // Initialize zmq context
    zmq::context_t context(1);
    // zmq::socket_t publisher(context, ZMQ_PUB);
    // publisher.bind("tcp://*:5555");
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:6666");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    // auto camera = Camera();
    // camera.configure(set);
    // camera.start();

    // // Print all parameters
    // std::cout << "Camera settings:" << std::endl;
    // std::cout << "Device Index: " << set.device_index << std::endl;
    // std::cout << "Resolution: " << camera.get_capture().get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camera.get_capture().get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    // std::cout << "FPS: " << camera.get_capture().get(cv::CAP_PROP_FPS) << std::endl;
    // std::cout << "Brightness: " << camera.get_capture().get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
    // std::cout << "Contrast: " << camera.get_capture().get(cv::CAP_PROP_CONTRAST) << std::endl;
    // std::cout << "Saturation: " << camera.get_capture().get(cv::CAP_PROP_SATURATION) << std::endl;
    // std::cout << "Hue: " << camera.get_capture().get(cv::CAP_PROP_HUE) << std::endl;
    // // std::cout << "Gain: " << camera.get_capture().get(cv::CAP_PROP_GAIN) << std::endl;

    init_command_handler();

    while (1) {
        // if (frame.empty()) {
        //     std::cout << "End of video stream" << std::endl;
        //     break;
        // }

        // Receive message
        zmq::message_t received;
        subscriber.recv(&received);
        uchar cam_number = *(static_cast<uchar*>(received.data()));
        std::cout << "Received header: " << int(cam_number) << std::endl;
        subscriber.recv(&received); // frame

        // temporarily make placeholder size and type
        cv::Mat frame(360, 640, CV_8UC3);
        cv::Mat new_frame(frame.size(), frame.type(), received.data());

        if (cam_number == 1) {
            cv::imshow("Frame1", new_frame);
        } else if (cam_number == 3) {
            cv::imshow("Frame3", new_frame);
        } else {
            cv::imshow("FrameERROR", new_frame);
        }

        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}

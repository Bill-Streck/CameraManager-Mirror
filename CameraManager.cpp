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
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:6666");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    init_command_handler();

    while (1) {
        // Receive message
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
    clean_command_handler(); // TODO make timeoutable

    return 0;
}

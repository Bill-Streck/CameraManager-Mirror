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

int main() {
    auto device_index = 0;

    auto set = settings();
    set.device_index = 0;
    set.width = 640;
    set.height = 360;
    set.fps = 30;

    // see what stuff does
    set.brightness = 50;
    set.contrast = 100;
    set.saturation = 100;
    set.hue = 100;
    set.gain = 100;
    set.exposure = 200;
    set.gamma = 100;

    // Initialize zmq context
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");

    auto camera = Camera(front);
    camera.configure(set);
    camera.start();

    while (1) {
        // cv::Mat frame;
        // cap >> frame;

        auto frame = camera.get_current_frame();

        if (frame.empty()) {
            std::cout << "End of video stream" << std::endl;
            break;
        }

        // Frame to zmq
        // TODO see how this data actually appears
        zmq::message_t message(frame.data, frame.total() * frame.elemSize(), NULL);
        publisher.send(message);
        
        

        cv::imshow("Frame", frame);

        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }

    // cap.release();
    camera.stop_all();

    cv::destroyAllWindows();

    return 0;
}

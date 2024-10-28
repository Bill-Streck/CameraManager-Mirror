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

    // // Query available cameras
    // std::cout << "Available cameras:" << std::endl;
    // for (int device_index = 0; device_index < 10; ++device_index) {
    //     cv::VideoCapture cap(device_index);
    //     if (cap.isOpened()) {
    //         std::cout << "Camera " << device_index << " is available." << std::endl;
    //         cap.release();
    //     } else {
    //         std::cout << "Camera " << device_index << " is not available." << std::endl;
    //     }
    // }

    // test call
    // create_compressed_stream();

    auto device_index = 0;

    auto set = settings();
    set.device_index = 0;
    set.width = 640;
    set.height = 360;
    set.fps = 30;

    // see what stuff does
    set.brightness = 50;
    set.contrast = 50;
    set.saturation = 50;
    set.hue = 50;
    set.gain = 50;
    set.exposure = 50;
    set.gamma = 50;

    // Initialize zmq context
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5555");
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // Subscribe to all messages

    auto camera = Camera(front);
    camera.configure(set);
    camera.start();

    // double format = camera.get_capture().get(cv::CAP_PROP_FORMAT);
    // std::cout << "Default Pixel Format: " << format << std::endl;

    avif_init();

    auto count = 0;

    while (1) {
        auto frame = camera.get_current_frame();

        std::vector<uchar> buf;

        count++;

        // compress_to_avif(frame, buf);

        if (frame.empty()) {
            std::cout << "End of video stream" << std::endl;
            break;
        }

        // publish message
        zmq::message_t message(frame.data, frame.total() * frame.elemSize());
        publisher.send(message, frame.total() * frame.elemSize());

        std::cout << "Frame " << count << " sent";

        // Receive message
        zmq::message_t received;
        subscriber.recv(&received);

        std::cout << "     Frame " << count << " received" << std::endl;

        // copy message to new frame
        // TOOD find a way to not depend on frame.size(), frame.type() (or just know them beforehand?)
        cv::Mat new_frame(frame.size(), frame.type(), received.data());

        // relay gains in compression in console
        // float ogsize = frame.size().height * frame.size().width;
        // float newsize = buf.size();
        // std::cout << "Original frame size: " << ogsize << std::endl;
        // std::cout << "Compressed frame size: " << newsize << "    Savings: " << (1-newsize/ogsize) * 100 << "%" << std::endl;
        
        // cv::imshow("Frame", frame);
        // cv::imshow("Frame", decompress_from_avif(buf));
        cv::imshow("Frame", new_frame);

        char c = (char) cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }

    avif_cleanup();

    camera.stop_all();

    cv::destroyAllWindows();

    return 0;
}

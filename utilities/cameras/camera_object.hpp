/**
 * @file camera_object.hpp
 * @author William Streck
 * @brief Camera object definition.
 * @version 0.1
 * @date 2024-10-24
 * 
 */

#ifndef CAMERA_OBJECT_HPP
#define CAMERA_OBJECT_HPP

#include "startup.hpp"

enum camera_id {
    front = 5,
    back = 6,
    TEST = 8
    // TODO the actual values
};

/**
 * @brief Camera wrapper class. Allows general management of camera utilities.
 * 
 */
class Camera {
    public:
        /**
         * @brief Construct a new Camera object
         * 
         * @param idx camera index based on dev rules
         */
        Camera(camera_id idx);
        
        /**
         * @brief Configures the camera with the given settings.
         * Preserves the current state of the camera.
         * 
         * @param set settings object with the desired camera settings.
         */
        void configure(settings set);

        /**
         * @brief Checks if the camera is ready to start.
         * 
         * @return true if the camera is ready to start.
         * @return false if the camera is not ready to start.
         */
        bool ready();

        /**
         * @brief Starts the camera.
         * 
         * @return true if the camera started successfully.
         * @return false if the camera did not start successfully.
         */
        bool start();

        /**
         * @brief Streams the camera.
         * 
         * @return true if the camera started streaming successfully.
         * @return false if the camera did not start streaming successfully.
         */
        bool stream();

        /**
         * @brief Gets the current frame from the camera.
         * 
         * @return cv::Mat* pointer to the current frame.
         */
        cv::Mat get_current_frame();

        /**
         * @brief Stops all camera operations.
         * 
         */
        void stop_all();

        /**
         * @brief Stops the camera stream.
         * 
         */
        void stop_stream();

        /**
         * @brief Hard resets the camera.
         * 
         */
        void hard_reset();
    private:
        cv::VideoCapture cap;
        settings set;
        bool ready_start = false;
        bool running = false;
        bool streaming = false;
};

#endif

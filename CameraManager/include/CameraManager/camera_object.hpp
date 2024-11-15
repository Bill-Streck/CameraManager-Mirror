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
        Camera();

        /**
         * @brief Get the device index object
         * 
         * @return int the device index.
         */
        int get_device_index();
        
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
         * Unless settings were already modified, this will restore defaults.
         * 
         * @return true if the camera started successfully.
         * @return false if the camera did not start successfully.
         */
        bool start();

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
         * @brief Changes an attribute on the camera within those identified.
         * 
         * @param attribute Attribute key to change.
         * @param modifier Amount to change attribute by, or ignored if not applicable (white balance).
         * @returns result of the change as identified by cap.set()
         */
        bool change_attribute(int attribute, int modifier);

        /**
         * @brief Hard resets the camera.
         * 
         */
        void hard_reset();

        /**
         * @brief Gets the camera capture object.
         * 
         * @return cv::VideoCapture the camera capture object.
         */
        cv::VideoCapture get_capture();
    private:
        cv::VideoCapture cap;
        settings set;
        bool ready_start = false;
        bool running = false;
};

#endif
